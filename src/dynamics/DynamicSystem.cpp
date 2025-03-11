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


void DynamicSystem::step(const double dt) {
  if (m_particles.empty()) return;

  // Define a state structure for RK4
  struct State {
    Vector3d position;
    Vector3d velocity;
  };

  // Store initial states
  std::vector<State> initialStates;
  for (const auto &[id, particle] : m_particles) {
    initialStates.push_back({particle->getPosition(), particle->getVelocity()});
  }

  // Lambda function to evaluate forces
  auto evaluateForces = [this]() {
    VectorXd forces;
    buildForceVector(forces);
    return forces;
  };

  // Lambda function to compute derivatives (velocity and acceleration)
  auto computeDerivatives = [this](const std::vector<State> &states, const VectorXd &forces) {
    std::vector<State> derivatives;
    int i = 0;
    for (const auto &[id, particle] : m_particles) {
      Vector3d acc = forces.segment<3>(i * 3) / particle->getMass();
      derivatives.push_back({states[i].velocity, acc}); // Derivative of position is velocity, derivative of velocity is acceleration
      ++i;
    }
    return derivatives;
  };

  // --- RK4 Stages ---

  // Stage 1: Compute k1
  for (auto &[id, particle] : m_particles) particle->clearForces();
  for (auto &generator : m_forceGenerators) generator->apply(dt);
  VectorXd k1Forces = evaluateForces();
  auto k1Derivatives = computeDerivatives(initialStates, k1Forces);

  // Stage 2: Compute k2
  std::vector<State> k2States;
  for (size_t i = 0; i < initialStates.size(); ++i) {
    k2States.push_back({
      initialStates[i].position + 0.5 * dt * k1Derivatives[i].position,
      initialStates[i].velocity + 0.5 * dt * k1Derivatives[i].velocity
    });
  }
  for (auto &[id, particle] : m_particles) particle->clearForces();
  for (auto &generator : m_forceGenerators) generator->apply(dt);
  VectorXd k2Forces = evaluateForces();
  auto k2Derivatives = computeDerivatives(k2States, k2Forces);

  // Stage 3: Compute k3
  std::vector<State> k3States;
  for (size_t i = 0; i < initialStates.size(); ++i) {
    k3States.push_back({
      initialStates[i].position + 0.5 * dt * k2Derivatives[i].position,
      initialStates[i].velocity + 0.5 * dt * k2Derivatives[i].velocity
    });
  }
  for (auto &[id, particle] : m_particles) particle->clearForces();
  for (auto &generator : m_forceGenerators) generator->apply(dt);
  VectorXd k3Forces = evaluateForces();
  auto k3Derivatives = computeDerivatives(k3States, k3Forces);

  // Stage 4: Compute k4
  std::vector<State> k4States;
  for (size_t i = 0; i < initialStates.size(); ++i) {
    k4States.push_back({
      initialStates[i].position + dt * k3Derivatives[i].position,
      initialStates[i].velocity + dt * k3Derivatives[i].velocity
    });
  }
  for (auto &[id, particle] : m_particles) particle->clearForces();
  for (auto &generator : m_forceGenerators) generator->apply(dt);
  VectorXd k4Forces = evaluateForces();
  auto k4Derivatives = computeDerivatives(k4States, k4Forces);

  // --- Combine Results ---
  int i = 0;
  for (auto &[id, particle] : m_particles) {
    if (!particle->isFixed()) {
      Vector3d newPos = initialStates[i].position + (dt / 6.0) *
          (k1Derivatives[i].position + 2.0 * k2Derivatives[i].position +
           2.0 * k3Derivatives[i].position + k4Derivatives[i].position);
      Vector3d newVel = initialStates[i].velocity + (dt / 6.0) *
          (k1Derivatives[i].velocity + 2.0 * k2Derivatives[i].velocity +
           2.0 * k3Derivatives[i].velocity + k4Derivatives[i].velocity);
      particle->setPosition(newPos);
      particle->setVelocity(newVel);
    }
    ++i;
  }

  // --- Constraint Stabilization ---
  solvePositionConstraints(
    1e-6,    // epsilon
    10,       // maxIterations
    1.0,     // alpha
    1e-6,    // lambda
    0.05     // maxCorrection
  );

  solveVelocityConstraints(
    1e-6,    // epsilon
    10,       // maxIterations
    1.0,     // alpha
    1e-6,    // lambda
    0.05     // maxCorrection
  );
  /*
  // --- Velocity Update Based on Corrected Positions ---
  i = 0;
  for (auto &[id, particle] : m_particles) {
    if (!particle->isFixed()) {
      Vector3d correctedVelocity = (particle->getPosition() - initialStates[i].position) / dt;
      particle->setVelocity(correctedVelocity);
    }
    ++i;
  }
  */
}
} // namespace Neutron
