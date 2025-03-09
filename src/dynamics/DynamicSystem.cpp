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

void DynamicSystem::addConstraint(const std::shared_ptr<Constraint>& constraint) {
  m_constraints.push_back(constraint);
  m_constraintSolver.addConstraint(constraint);
}

void DynamicSystem::clearConstraints() {
  m_constraints.clear();
  m_constraintSolver.clearConstraints();
}

void DynamicSystem::addForceGenerator(const std::shared_ptr<ForceGenerator>& generator) {
  m_forceGenerators.push_back(generator);
}

void DynamicSystem::buildMassMatrix() {
  int n = static_cast<int>(m_particles.size()) * 3; // 3 DOF per particle
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
  int n = static_cast<int>(m_particles.size()) * 3; // 3 DOF per particle
  forces.resize(n);
  forces.setZero();

  int i = 0;
  for (const auto &[id, particle]: m_particles) {
    forces.segment<3>(i * 3) = particle->getForce();
    ++i;
  }
  LOG_DEBUG("Force vector: \n", forces);
}

// Replace the existing step method in DynamicSystem.cpp with this implementation:
void DynamicSystem::step(const double dt) {
  if (m_particles.empty()) return;

  // Clear accumulated forces from previous frame
  for (auto& [id, particle] : m_particles) {
    particle->clearForces();
  }

  // Apply all force generators (springs, gravity, etc.)
  for (auto& generator : m_forceGenerators) {
    generator->apply(dt);
  }

  // Build mass matrix if needed
  if (m_massMatrix.size() == 0) {
    buildMassMatrix();
  }

  // Use RK4 integration
  integrateRK4(dt);
}

void DynamicSystem::setSystemState(const VectorXd& state) {
  int i = 0;
  for (auto& [id, particle] : m_particles) {
    if (!particle->isFixed()) {
      // Update position
      particle->setPosition(state.segment<3>(i * 6));
      // Update velocity
      particle->setVelocity(state.segment<3>(i * 6 + 3));
    }
    ++i;
  }
}

// Add these new method implementations to DynamicSystem.cpp:
void DynamicSystem::getSystemState(VectorXd& state) {
  // For each particle, we need 6 values (3 for position, 3 for velocity)
  int n = static_cast<int>(m_particles.size()) * 6;
  state.resize(n);

  int i = 0;
  for (const auto& [id, particle] : m_particles) {
    // Position (x, y, z)
    state.segment<3>(i * 6) = particle->getPosition();
    // Velocity (vx, vy, vz)
    state.segment<3>(i * 6 + 3) = particle->getVelocity();
    ++i;
  }
}

void DynamicSystem::calculateDerivatives(const VectorXd& state, VectorXd& derivatives) {
  // Temporarily save the current state
  VectorXd originalState;
  getSystemState(originalState);

  // Set the system to the provided state
  setSystemState(state);

  // Clear forces and apply force generators
  for (auto& [id, particle] : m_particles) {
    particle->clearForces();
  }

  for (auto& generator : m_forceGenerators) {
    generator->apply(0.0); // dt = 0 because we're not advancing time, just calculating forces
  }

  // Build force vector with current forces
  VectorXd forces;
  buildForceVector(forces);

  // Convert particles to vector for constraint solving
  std::vector<Particle*> particles;
  for (const auto& [id, particle] : m_particles) {
    particles.push_back(particle.get());
  }

  // Solve constraints
  MatrixXd jacobian;
  VectorXd constraintRHS;
  m_constraintSolver.buildJacobian(particles, jacobian, constraintRHS);

  VectorXd accelerations;
  VectorXd lambdas;
  m_constraintSolver.solveConstrainedSystem(
      m_massMatrix, forces, jacobian, constraintRHS, accelerations, lambdas
  );

  // Resize derivatives vector
  derivatives.resize(state.size());

  // For each particle, derivatives are [velocity, acceleration]
  int i = 0;
  for (const auto& [id, particle] : m_particles) {
    // Position derivatives = velocity
    derivatives.segment<3>(i * 6) = particle->getVelocity();

    // Velocity derivatives = acceleration
    if (!particle->isFixed()) {
      derivatives.segment<3>(i * 6 + 3) = accelerations.segment<3>(i * 3);
    } else {
      derivatives.segment<3>(i * 6 + 3).setZero();
    }
    ++i;
  }

  // Restore original state
  setSystemState(originalState);
}

void DynamicSystem::integrateRK4(const double dt) {
  // Get current state
  VectorXd state;
  getSystemState(state);

  // Calculate the four RK4 increments
  VectorXd k1, k2, k3, k4;

  // k1 = f(yn, tn)
  calculateDerivatives(state, k1);

  // k2 = f(yn + dt/2 * k1, tn + dt/2)
  VectorXd tempState = state + (dt * 0.5) * k1;
  calculateDerivatives(tempState, k2);

  // k3 = f(yn + dt/2 * k2, tn + dt/2)
  tempState = state + (dt * 0.5) * k2;
  calculateDerivatives(tempState, k3);

  // k4 = f(yn + dt * k3, tn + dt)
  tempState = state + dt * k3;
  calculateDerivatives(tempState, k4);

  // Update state: yn+1 = yn + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
  VectorXd newState = state + (dt/6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);

  // Apply the new state to all particles
  setSystemState(newState);
}

} // namespace Neutron
