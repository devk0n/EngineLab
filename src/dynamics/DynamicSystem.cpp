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

Particle *DynamicSystem::getParticle(UniqueID id) {
  auto it = m_particles.find(id);
  if (it != m_particles.end()) {
    return it->second.get();
  }
  return nullptr;
}

void DynamicSystem::addConstraint(std::shared_ptr<Constraint> constraint) {
  m_constraints.push_back(constraint);
  m_constraintSolver.addConstraint(constraint);
}

void DynamicSystem::clearConstraints() {
  m_constraints.clear();
  m_constraintSolver.clearConstraints();
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

void DynamicSystem::step(double dt) {
  if (m_particles.empty()) return;

  // Build mass matrix (if not already built)
  if (m_massMatrix.size() == 0) {
    buildMassMatrix();
  }

  // Build force vector (must be done every step)
  VectorXd forces;
  buildForceVector(forces);

  // Convert m_particles to a vector of Particle*
  std::vector<Particle*> particles;
  for (const auto& [id, particle] : m_particles) {
    particles.push_back(particle.get());
  }

  // Build constraint Jacobian and RHS
  MatrixXd jacobian;
  VectorXd constraintRHS;
  m_constraintSolver.buildJacobian(particles, jacobian, constraintRHS, 1000.0, 1000.0);

  // Solve the constrained system using the two-step model
  VectorXd accelerations;
  VectorXd lambdas;
  m_constraintSolver.solveConstrainedSystem(
      m_massMatrix, forces, jacobian, constraintRHS, accelerations, lambdas
  );

  // Update velocities and positions
  int i = 0;
  for (const auto& [id, particle] : m_particles) {
    Vector3d acc = accelerations.segment<3>(i * 3);
    if (!particle->isFixed()) {
      particle->setVelocity(particle->getVelocity() + acc * dt);
      particle->setPosition(particle->getPosition() + particle->getVelocity() * dt);
    }
    ++i;
  }
}
} // namespace Neutron
