#include "DynamicSystem.h"

namespace Neutron {
DynamicSystem::DynamicSystem() = default;

void DynamicSystem::buildMassInertiaTensor() {
  if (!m_dirty) return;

  int n = static_cast<int>(m_bodies.size()) * 6;
  m_massInertiaTensor.resize(n);
  m_massInertiaTensor.setZero();

  int i = 0;
  for (const auto &body: m_bodies) {
    m_massInertiaTensor.segment<3>(i * 6).setConstant(body->getMass());
    m_massInertiaTensor.segment<3>(i * 6 + 3) = body->getInertia();
    ++i;
  }

  m_dirty = false;
}

void DynamicSystem::buildWrench(VectorXd &wrench) const {
  int n = static_cast<int>(m_bodies.size()) * 6; // 6 DOF per body
  wrench.resize(n);
  wrench.setZero();

  int i = 0;
  for (const auto &body: m_bodies) {
    wrench.segment<3>(i * 6) = body->getForce();
    wrench.segment<3>(i * 6 + 3) = body->getTorque(); -
                                   (skew(body->getAngularVelocity()) * body->
                                    getInertia().asDiagonal() * body->
                                    getAngularVelocity());
    ++i;
  }
}

void DynamicSystem::buildConstraints(
  VectorXd &phi,
  MatrixXd &jacobian,
  VectorXd &gamma,
  VectorXd &accelerations,
  VectorXd &lambdas) {

  // Resize matrices to accommodate all constraints
  phi.resize(m_numConstraints);
  jacobian.resize(m_numConstraints, m_numBodies * 6);
  gamma.resize(m_numConstraints);
  accelerations.resize(m_numBodies * 6);
  lambdas.resize(m_numConstraints);

  // Zero-initialize the matrices
  phi.setZero();
  jacobian.setZero();
  gamma.setZero();
  accelerations.setZero();
  lambdas.setZero();

  // Collect qdot (velocities)
  VectorXd qdot(m_bodies.size() * 6);
  for (int i = 0; i < m_bodies.size(); ++i) {
    qdot.segment<3>(i * 6) = m_bodies[i]->getVelocity();
  }

  // Current row in the Jacobian and constraintRHS
  int startRow = 0;
  for (const auto& constraint : m_constraints) {

    constraint->computePhi(phi, startRow);
    constraint->computeJacobian(jacobian, startRow);
    constraint->computeGamma(gamma, startRow);

    startRow += constraint->getDOFs();
  }
}

void DynamicSystem::step(const double dt) {
  if (m_bodies.empty()) return;
  buildMassInertiaTensor();

  // --- 1. Save current state ---
  VectorXd q_prev, q_dot_prev;
  getSystemState(q_prev);
  getVelocityState(q_dot_prev);

  // --- 2. Predict midpoint state ---
  VectorXd q_mid = q_prev + 0.5 * dt * q_dot_prev;
  VectorXd q_dot_mid = q_dot_prev + 0.5 * dt * computeAccelerations(q_prev, q_dot_prev);

  // --- 3. Solve at midpoint ---
  VectorXd forces, phi, gamma, accelerations, lambdas;
  MatrixXd jacobian;

  // Set system to midpoint state
  setSystemState(q_mid);
  setVelocityState(q_dot_mid);

  // Compute forces and constraints at midpoint
  buildWrench(forces);
  buildConstraints(phi, jacobian, gamma, accelerations, lambdas);

  // Remove Baumgarte terms from gamma (keep original constraint derivatives)
  gamma = jacobian * q_dot_mid; // Simplified for illustration

  // Solve augmented system
  m_solver.solveSystem(
    m_massInertiaTensor,
    forces,
    phi,
    jacobian,
    gamma,
    accelerations,
    lambdas
  );

  // --- 4. Update state with midpoint derivatives ---
  VectorXd q_new = q_prev + dt * (q_dot_prev + 0.5 * dt * accelerations);
  VectorXd q_dot_new = q_dot_prev + dt * accelerations;

  // --- 5. Project positions ---

  // --- 6. Project velocities ---

  // --- 7. Update final state ---
  setSystemState(q_new);
  setVelocityState(q_dot_new);
}

void DynamicSystem::getVelocityState(VectorXd& velocities) const {
  velocities.resize(m_bodies.size() * 6);
  for (size_t i = 0; i < m_bodies.size(); ++i) {
    velocities.segment<3>(i*6) = m_bodies[i]->getVelocity();
    velocities.segment<3>(i*6+3) = m_bodies[i]->getAngularVelocity();
  }
}

void DynamicSystem::setVelocityState(const VectorXd& velocities) const {
  for (size_t i = 0; i < m_bodies.size(); ++i) {
    m_bodies[i]->setVelocity(velocities.segment<3>(i*6));
    m_bodies[i]->setAngularVelocity(velocities.segment<3>(i*6+3));
  }
}


void DynamicSystem::setSystemState(VectorXd state) {
  int i = 0;
  for (auto &body: m_bodies) {
    body->setPosition(state.segment<3>(i * 7));
    body->setOrientation(state.segment<4>(i * 7 + 3));
    ++i;
  }
}

void DynamicSystem::getSystemState(VectorXd& state) const {
  state.resize(m_numBodies * 7);

  int i = 0;
  for (const auto &body: m_bodies) {
    state.segment<3>(i * 7) = body->getPosition();
    state.segment<4>(i * 7 + 3) = body->getOrientation();
    ++i;
  }
}

UniqueID DynamicSystem::addBody(
  const double &mass,
  const Vector3d &inertia,
  const Vector3d &position,
  const Vector4d &orientation
) {
  UniqueID ID = m_nextID++;
  m_bodies.emplace_back(std::make_unique<Body>(
    ID,
    m_numBodies,
    mass,
    inertia,
    position,
    orientation.normalized()
  ));
  m_bodyIndex.try_emplace(ID, m_bodies.size() - 1);
  m_dirty = true;
  m_numBodies++;
  return ID;
}

Body *DynamicSystem::getBody(const UniqueID ID) {
  auto it = m_bodyIndex.find(ID);
  if (it != m_bodyIndex.end()) {
    // Check if the index is valid
    if (it->second < m_bodies.size()) {
      return m_bodies[it->second].get();
    }
  }
  return nullptr; // Return nullptr if the body is not found
}

const Body *DynamicSystem::getBody(const UniqueID ID) const {
  auto it = m_bodyIndex.find(ID);
  if (it != m_bodyIndex.end()) {
    // Check if the index is valid
    if (it->second < m_bodies.size()) {
      return m_bodies[it->second].get();
    }
  }
  return nullptr; // Return nullptr if the body is not found
}

void DynamicSystem::addConstraint(
  const std::shared_ptr<Constraint> &constraint) {
  m_constraints.emplace_back(constraint);
  m_numConstraints += constraint->getDOFs();
}

const std::vector<std::shared_ptr<Constraint> > &
DynamicSystem::getConstraints() const {
  return m_constraints;
}
} // namespace Neutron
