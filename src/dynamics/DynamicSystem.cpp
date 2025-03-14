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

void DynamicSystem::buildWrench(VectorXd &wrench) {
  int n = static_cast<int>(m_bodies.size()) * 6; // 6 DOF per body
  wrench.resize(n);
  wrench.setZero();

  int i = 0;
  for (const auto &body: m_bodies) {
    wrench.segment<3>(i * 6) = body->getForce();
    wrench.segment<3>(i * 6 + 3) = body->getTorque() -
                                   (skew(body->getAngularVelocity()) * body->
                                    getInertia().asDiagonal() * body->
                                    getAngularVelocity());
    ++i;
  }
}

void DynamicSystem::buildConstraints(
  MatrixXd &jacobian,
  VectorXd &gamma) {
  // Resize matrices to accommodate all constraints
  jacobian.resize(m_numConstraints, m_numBodies * 6);
  gamma.resize(m_numConstraints);

  // Zero-initialize the matrices
  jacobian.setZero();
  gamma.setZero();

  // Have each constraint fill its portion of the matrices
  int rowOffset = 0;
  for (const auto &constraint: m_constraints) {
    constraint->computeJacobian(jacobian, rowOffset);
    constraint->computeGamma(gamma, rowOffset);
    rowOffset += constraint->getDOFs();
  }

  // LOG_DEBUG("Jacobian: \n", jacobian);
}

Matrix4d idk(Vector3d w) {
  Eigen::Matrix4d W;
  W <<     0, -w.x(), -w.y(), -w.z(),
       w.x(),      0,  w.z(), -w.y(),
       w.y(), -w.z(),      0,  w.x(),
       w.z(),  w.y(), -w.x(),      0;
  return W;
}

void DynamicSystem::step(double dt) {
  if (m_bodies.empty()) return;
  buildMassInertiaTensor();

  // --- 1. Clear Forces and Apply External Forces ---
  for (auto &body: m_bodies) {
    body->clearForces();
    body->clearTorques();
  }

  for (auto &generator: m_forceGenerators) {
    generator->apply(dt);
  }

  // --- 2. Assemble System and Solve ---
  VectorXd forces, gamma, accelerations, lambdas;
  MatrixXd jacobian;
  buildWrench(forces);
  buildConstraints(jacobian, gamma);

  m_solver.solveSystem(
    m_massInertiaTensor,
    forces,
    jacobian,
    gamma,
    accelerations,
    lambdas
  );

  int i = 0;
  for (auto &body: m_bodies) {
    // Linear motion
    body->setVelocity(body->getVelocity() + accelerations.segment<3>(i * 6) * dt);
    body->setPosition(body->getPosition() + body->getVelocity() * dt);

    // Angular motion
    body->setAngularVelocity(body->getAngularVelocity() + accelerations.segment<3>(i * 6 + 3) * dt);

    auto quatVel = 0.5 * idk(body->getAngularVelocity()) * body->getOrientation();

    body->setOrientation(body->getOrientation() + quatVel * dt);

    ++i;
  }
}

UniqueID DynamicSystem::addBody(
  const double &mass,
  const Vector3d &inertia,
  const Vector3d &position,
  const Vector4d &orientation
) {
  // Input validation
  if (mass <= 0) {
    LOG_ERROR("Mass must be positive");
    return -1;
  }
  if (inertia.x() <= 0 || inertia.y() <= 0 || inertia.z() <= 0) {
    LOG_ERROR("Inertia components must be positive");
    return -1;
  }

  UniqueID ID = m_nextID++;
  m_bodies.emplace_back(std::make_unique<Body>(
    ID,
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

Body *DynamicSystem::getBody(UniqueID ID) {
  auto it = m_bodyIndex.find(ID);
  if (it != m_bodyIndex.end()) {
    // Check if the index is valid
    if (it->second < m_bodies.size()) {
      return m_bodies[it->second].get();
    }
  }
  return nullptr; // Return nullptr if the body is not found
}

const Body *DynamicSystem::getBody(UniqueID ID) const {
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
