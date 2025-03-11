// dynamics/DynamicSystem.cpp
#include "DynamicSystem.h"
#include <algorithm>
#include "utils/Logger.h"

namespace Neutron {
DynamicSystem::DynamicSystem() = default;

UniqueID DynamicSystem::addParticle(
  double mass,
  const Vector3d &position) {
  UniqueID id = m_nextID++;
  auto particle = std::make_unique<Particle>(id, mass, position);
  m_particles[id] = std::move(particle);
  return id;
}

Particle *DynamicSystem::getParticle(const UniqueID id) {
  if (auto it = m_particles.find(id); it != m_particles.end()) {
    return it->second.get();
  }
  return nullptr;
}

void DynamicSystem::addConstraint(
  const std::shared_ptr<Constraint> &constraint) {
  m_constraints.push_back(constraint);
  m_constraintSolver.addConstraint(constraint);
}

void DynamicSystem::clearConstraints() {
  m_constraints.clear();
  m_constraintSolver.clearConstraints();
}

void DynamicSystem::addForceGenerator(
  const std::shared_ptr<ForceGenerator> &generator) {
  m_forceGenerators.push_back(generator);
}

void DynamicSystem::buildMassMatrix() {
  int n = m_particles.size() * 3; // 3 DOF per particle
  m_massMatrix.resize(n);
  m_massMatrix.setZero();

  int i = 0;
  for (const auto &[id, particle]: m_particles) {
    m_massMatrix.segment<3>(i * 3).setConstant(particle->getMass());
    ++i;
  }
  LOG_DEBUG("Mass matrix: \n", m_massMatrix);
}

void DynamicSystem::buildForceVector(VectorXd &forces) {
  int n = m_particles.size() * 3; // 3 DOF per particle
  forces.resize(n);
  forces.setZero();

  int i = 0;
  for (const auto &[id, particle]: m_particles) {
    forces.segment<3>(i * 3) = particle->getForce();
    ++i;
  }
  LOG_DEBUG("Force vector: \n", forces);
}

void DynamicSystem::solvePositionConstraints(
    const double epsilon = 1e-6,
    const int maxIterations = 10,
    const double alpha = 0.5,         // Relaxation parameter
    const double lambda = 1e-6,       // Use damped least squares
    const double maxCorrection = 0.1) // Maximum velocity correction per iteration
{
  if (m_particles.empty()) return;

  std::vector<Particle *> particles;
  for (const auto &[id, particle]: m_particles) {
    particles.push_back(particle.get());
  }

  // Smaller value = more stable but slower convergence
  for (int iter = 0; iter < maxIterations; ++iter) {
    // Build Jacobian and constraint equations
    MatrixXd jacobian;
    VectorXd constraintRHS;
    m_constraintSolver.buildJacobian(particles, jacobian, constraintRHS);

    // Calculate constraint violations
    VectorXd c(constraintRHS.size());
    int itt = 0;
    for (const auto &constraint: m_constraints) {
      constraint->computeConstraintEquations(c, itt);
      ++itt;
    }

    // Check if constraints are satisfied
    double error = c.norm();
    if (error < epsilon) {
      break;
    }

    // Use damped least squares (Levenberg-Marquardt)
    MatrixXd JTJ = jacobian.transpose() * jacobian;
    MatrixXd H = JTJ + lambda * MatrixXd::Identity(JTJ.rows(), JTJ.cols());
    VectorXd dx = H.ldlt().solve(jacobian.transpose() * (-c));

    // Apply scaled corrections to positions
    int i = 0;
    for (auto &particle: particles) {
      if (!particle->isFixed()) {
        Vector3d correction = alpha * dx.segment<3>(i * 3);

        // Limit correction magnitude
        if (correction.norm() > maxCorrection) {
          correction *= maxCorrection / correction.norm();
        }
        particle->setPosition(particle->getPosition() + correction);
      }
      i++;
    }
  }
}

void DynamicSystem::solveVelocityConstraints(
    const double epsilon = 1e-6,
    const int maxIterations = 10,
    const double alpha = 0.5,         // Relaxation parameter
    const double lambda = 1e-6,       // Use damped least squares
    const double maxCorrection = 0.1) // Maximum velocity correction per iteration
{
  if (m_particles.empty()) return;

  std::vector<Particle *> particles;
  for (const auto &[id, particle]: m_particles) {
    particles.push_back(particle.get());
  }

  for (int iter = 0; iter < maxIterations; ++iter) {
    // Build Jacobian
    MatrixXd jacobian;
    VectorXd dummy;
    m_constraintSolver.buildJacobian(particles, jacobian, dummy);

    // Compute current velocity constraint violation
    VectorXd qdot(particles.size() * 3);
    for (int i = 0; i < particles.size(); ++i) {
      qdot.segment<3>(i * 3) = particles[i]->getVelocity();
    }

    VectorXd cdot = jacobian * qdot;

    // Check if velocity constraints are satisfied
    if (cdot.norm() < epsilon) {
      break;
    }

    MatrixXd JTJ = jacobian.transpose() * jacobian;
    MatrixXd H = JTJ + lambda * MatrixXd::Identity(JTJ.rows(), JTJ.cols());
    VectorXd dv = H.ldlt().solve(jacobian.transpose() * (-cdot));

    // Apply scaled corrections to velocities
    int i = 0;
    for (auto &particle: particles) {
      if (!particle->isFixed()) {
        Vector3d correction = alpha * dv.segment<3>(i * 3);

        // Limit correction magnitude
        if (correction.norm() > maxCorrection) {
          correction *= maxCorrection / correction.norm();
        }
        particle->setVelocity(particle->getVelocity() + correction);
      }
      i++;
    }
  }
}

// NEW INTEGRATOR IMPLEMENTATION
void DynamicSystem::step(const double dt) {
  if (m_particles.empty()) return;

  // --- Stage 1: Velocity Verlet Position Update ---
  std::vector<Vector3d> initialPositions;
  std::vector<Vector3d> initialVelocities;
  VectorXd initialForces;

  // Store initial state
  for (const auto &[id, particle] : m_particles) {
    initialPositions.push_back(particle->getPosition());
    initialVelocities.push_back(particle->getVelocity());
  }

  // Compute initial forces
  buildForceVector(initialForces);

  // Update positions
  int i = 0;
  for (auto &[id, particle] : m_particles) {
    if (!particle->isFixed()) {
      Vector3d acc = initialForces.segment<3>(i*3) / particle->getMass();
      Vector3d newPos = particle->getPosition() +
                       particle->getVelocity() * dt +
                       0.5 * acc * dt * dt;
      particle->setPosition(newPos);
    }
    ++i;
  }

  // --- Stage 2: Compute New Forces ---
  for (auto &[id, particle] : m_particles) particle->clearForces();
  for (auto &generator : m_forceGenerators) generator->apply(dt);

  VectorXd newForces;
  buildForceVector(newForces);

  // --- Stage 3: Velocity Update ---
  i = 0;
  for (auto &[id, particle] : m_particles) {
    if (!particle->isFixed()) {
      Vector3d oldAcc = initialForces.segment<3>(i*3) / particle->getMass();
      Vector3d newAcc = newForces.segment<3>(i*3) / particle->getMass();
      Vector3d avgAcc = 0.5 * (oldAcc + newAcc);

      particle->setVelocity(initialVelocities[i] + avgAcc * dt);
    }
    ++i;
  }

  // --- Constraint Stabilization ---
  // MODIFIED: Added explicit parameters
  solvePositionConstraints(
    1e-6,    // epsilon
    3,       // maxIterations (increased)
    0.2,      // alpha (reduced from 0.5)
    1e-8,     // lambda (reduced damping)
    0.05      // maxCorrection
  );

  solveVelocityConstraints(
    1e-6,    // epsilon
    3,       // maxIterations
    0.2,      // alpha
    1e-8,     // lambda
    0.05      // maxCorrection
  );

}
} // namespace Neutron
