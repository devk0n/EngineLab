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

void DynamicSystem::solvePositionConstraints(double epsilon = 1e-6,
                                             int maxIterations = 10) {
  if (m_particles.empty()) return;

  std::vector<Particle *> particles;
  for (const auto &[id, particle]: m_particles) {
    particles.push_back(particle.get());
  }

  // Relaxation parameter to prevent overshooting
  const double alpha = 0.5;
  // Smaller value = more stable but slower convergence

  for (int iter = 0; iter < maxIterations; ++iter) {
    // Build Jacobian and constraint equations
    MatrixXd jacobian;
    VectorXd constraintRHS;
    m_constraintSolver.buildJacobian(particles, jacobian, constraintRHS);

    // Calculate constraint violations
    VectorXd c(constraintRHS.size());
    for (const auto &constraint: m_constraints) {
      constraint->computeConstraintEquations(c, 0);
    }

    // Check if constraints are satisfied
    double error = c.norm();
    if (error < epsilon) {
      break;
    }

    // Use damped least squares (Levenberg-Marquardt)
    const double lambda = 1e-6; // Damping factor
    MatrixXd JTJ = jacobian.transpose() * jacobian;
    MatrixXd H = JTJ + lambda * MatrixXd::Identity(JTJ.rows(), JTJ.cols());
    VectorXd dx = H.ldlt().solve(jacobian.transpose() * (-c));

    // Apply scaled corrections to positions
    int i = 0;
    for (auto &particle: particles) {
      if (!particle->isFixed()) {
        Vector3d correction = alpha * dx.segment<3>(i * 3);

        // Limit correction magnitude
        double maxCorrection = 0.1; // Maximum position correction per iteration
        if (correction.norm() > maxCorrection) {
          correction *= maxCorrection / correction.norm();
        }

        particle->setPosition(particle->getPosition() + correction);
      }
      i++;
    }
  }
}

void DynamicSystem::solveVelocityConstraints(double epsilon = 1e-6,
                                             int maxIterations = 10) {
  if (m_particles.empty()) return;

  std::vector<Particle *> particles;
  for (const auto &[id, particle]: m_particles) {
    particles.push_back(particle.get());
  }

  const double alpha = 0.5; // Relaxation parameter

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

    // Use damped least squares
    const double lambda = 1e-6;
    MatrixXd JTJ = jacobian.transpose() * jacobian;
    MatrixXd H = JTJ + lambda * MatrixXd::Identity(JTJ.rows(), JTJ.cols());
    VectorXd dv = H.ldlt().solve(jacobian.transpose() * (-cdot));

    // Apply scaled corrections to velocities
    int i = 0;
    for (auto &particle: particles) {
      if (!particle->isFixed()) {
        Vector3d correction = alpha * dv.segment<3>(i * 3);

        // Limit correction magnitude
        double maxCorrection = 1.0; // Maximum velocity correction per iteration
        if (correction.norm() > maxCorrection) {
          correction *= maxCorrection / correction.norm();
        }

        particle->setVelocity(particle->getVelocity() + correction);
      }
      i++;
    }
  }
}

void DynamicSystem::step(const double dt) {
  if (m_particles.empty()) return;

  // Store previous positions and velocities
  std::vector<Vector3d> prevPositions;
  std::vector<Vector3d> prevVelocities;
  for (const auto &[id, particle]: m_particles) {
    prevPositions.push_back(particle->getPosition());
    prevVelocities.push_back(particle->getVelocity());
  }

  // Clear and apply forces
  for (auto &[id, particle]: m_particles) {
    particle->clearForces();
  }
  for (auto &generator: m_forceGenerators) {
    generator->apply(dt);
  }

  // Build system matrices
  if (m_massMatrix.size() == 0) {
    buildMassMatrix();
  }
  VectorXd forces;
  buildForceVector(forces);

  // Solve dynamics
  std::vector<Particle *> particles;
  for (const auto &[id, particle]: m_particles) {
    particles.push_back(particle.get());
  }

  MatrixXd jacobian;
  VectorXd constraintRHS;
  m_constraintSolver.buildJacobian(particles, jacobian, constraintRHS);

  VectorXd accelerations;
  VectorXd lambdas;
  m_constraintSolver.solveConstrainedSystem(
    m_massMatrix, forces, jacobian, constraintRHS, accelerations, lambdas
  );

  // Semi-implicit Euler integration
  int i = 0;
  for (const auto &[id, particle]: m_particles) {
    if (!particle->isFixed()) {
      Vector3d acc = accelerations.segment<3>(i * 3);

      // Update velocity first
      Vector3d newVel = prevVelocities[i] + acc * dt;
      particle->setVelocity(newVel);

      // Then update position using new velocity
      Vector3d newPos = prevPositions[i] + newVel * dt;
      particle->setPosition(newPos);
    }
    ++i;
  }

  // Stabilize constraints with relaxation
  solvePositionConstraints(1e-6, 5); // Reduced iterations
  solveVelocityConstraints(1e-6, 5); // Reduced iterations
}
} // namespace Neutron
