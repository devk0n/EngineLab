#include "Dynamics.h"

namespace Proton {

VectorXd Dynamics::getPositionState() const {
  VectorXd positionState(m_numBodies * 3);

  int i = 0;
  for (const auto &body : m_bodies) {
    positionState.segment<3>(i * 3) = body->getPosition();
    ++i;
  }

  // LOG_DEBUG("Position State: \n", positionState);

  return positionState;
}

VectorXd Dynamics::getVelocityState() const {
  VectorXd velocityState(m_numBodies * 3);

  int i = 0;
  for (const auto &body : m_bodies) {
    velocityState.segment<3>(i * 3) = body->getLinearVelocity();
    ++i;
  }

  // LOG_DEBUG("Velocity State: \n", velocityState);

  return velocityState;
}

void Dynamics::step(double dt) {
  const int dof_q = m_numBodies * 7;   // 3 position + 4 quaternion (w, x, y, z)
  const int dof_dq = m_numBodies * 6;  // 3 linear + 3 angular velocity

  VectorXd q_n(dof_q), dq_n(dof_dq);
  for (int i = 0; i < m_numBodies; ++i) {
    const auto& body = m_bodies[i];
    q_n.segment<3>(i * 7) = body->getPosition();
    q_n.segment<4>(i * 7 + 3) = body->getOrientation(); // (w, x, y, z)
    dq_n.segment<3>(i * 6) = body->getLinearVelocity();
    dq_n.segment<3>(i * 6 + 3) = body->getAngularVelocity();
  }

  VectorXd q_next = q_n;
  VectorXd dq_next = dq_n;

  constexpr int maxIters = 10;
  constexpr double tol = 1e-8;

  for (int iter = 0; iter < maxIters; ++iter) {
    const VectorXd q_mid = 0.5 * (q_n + q_next);
    const VectorXd dq_mid = 0.5 * (dq_n + dq_next);

    // === Update body states to midpoint
    for (int i = 0; i < m_numBodies; ++i) {
      auto& body = m_bodies[i];
      body->clearForces();
      body->clearTorque();

      if (body->isFixed()) continue;

      body->setPosition(q_mid.segment<3>(i * 7));

      Vector4d q = q_mid.segment<4>(i * 7 + 3);
      q.normalize();
      body->setOrientation(q);

      body->setLinearVelocity(dq_mid.segment<3>(i * 6));
      body->setAngularVelocity(dq_mid.segment<3>(i * 6 + 3));
      body->updateInertiaWorld();
    }

    for (const auto& fg : m_forceGenerators) {
      fg->apply(dt);
    }

    // === External forces and torques
    VectorXd F_ext = VectorXd::Zero(dof_dq);
    for (int i = 0; i < m_numBodies; ++i) {
      const auto& body = m_bodies[i];
      F_ext.segment<3>(i * 6) = body->getForce();
      F_ext.segment<3>(i * 6 + 3) = body->getTorque();
    }

    // === Mass and inertia matrix
    MatrixXd M = MatrixXd::Zero(dof_dq, dof_dq);
    for (int i = 0; i < m_numBodies; ++i) {
      const auto& body = m_bodies[i];
      if (body->isFixed()) continue;
      M.block<3, 3>(i * 6, i * 6) = body->getMass() * Matrix3d::Identity();
      M.block<3, 3>(i * 6 + 3, i * 6 + 3) = body->getInertiaWorld();
    }

    // === Constraints
    MatrixXd P = MatrixXd::Zero(m_numConstraints, dof_dq);
    VectorXd gamma = VectorXd::Zero(m_numConstraints);
    int row = 0;
    for (const auto& c : m_constraints) {
      c->computeJacobian(P, row);
      c->computeAccelerationCorrection(gamma, row);
      row += c->getDOFs();
    }

    // === Solve KKT system
    MatrixXd KKT(dof_dq + m_numConstraints, dof_dq + m_numConstraints);
    KKT.setZero();
    KKT.topLeftCorner(dof_dq, dof_dq) = M;
    KKT.topRightCorner(dof_dq, m_numConstraints) = P.transpose();
    KKT.bottomLeftCorner(m_numConstraints, dof_dq) = P;

    VectorXd rhs(dof_dq + m_numConstraints);
    rhs.head(dof_dq) = F_ext;
    rhs.tail(m_numConstraints) = -gamma;

    VectorXd sol = KKT.completeOrthogonalDecomposition().solve(rhs);
    VectorXd ddq_mid = sol.head(dof_dq);

    // === Midpoint integration
    VectorXd dq_new = dq_n + dt * ddq_mid;
    VectorXd q_new = q_n;

    for (int i = 0; i < m_numBodies; ++i) {
      // Linear position update
      q_new.segment<3>(i * 7) += dt * 0.5 * (dq_n.segment<3>(i * 6) + dq_new.segment<3>(i * 6));

      // Quaternion update from angular velocity
      Vector4d q = q_n.segment<4>(i * 7 + 3);
      Vector3d omega = 0.5 * (dq_n.segment<3>(i * 6 + 3) + dq_new.segment<3>(i * 6 + 3));

      // dq/dt = 0.5 * Omega(omega) * q
      Vector4d dqdt;
      dqdt(0) = -0.5 * (omega.dot(q.segment<3>(1, 3))); // w'
      dqdt.segment<3>(1) = 0.5 * (q(0) * omega + omega.cross(q.segment<3>(1, 3))); // xyz'

      Vector4d q_updated = q + dt * dqdt;
      q_updated.normalize();
      q_new.segment<4>(i * 7 + 3) = q_updated;
    }

    double err = (dq_new - dq_next).norm() + (q_new - q_next).norm();
    dq_next = dq_new;
    q_next = q_new;
    if (err < tol) break;
  }

  // === 🔧 Position projection
  {
    VectorXd phi(m_numConstraints);
    MatrixXd J(m_numConstraints, dof_dq);
    int row = 0;
    for (const auto& c : m_constraints) {
      c->computePositionError(phi, row);
      c->computeJacobian(J, row);
      row += c->getDOFs();
    }

    if (m_numConstraints > 0) {
      MatrixXd JJt = J * J.transpose();
      VectorXd correction = J.transpose() * JJt.completeOrthogonalDecomposition().solve(phi);
      for (int i = 0; i < m_numBodies; ++i) {
        if (m_bodies[i]->isFixed()) continue;

        // Position correction
        q_next.segment<3>(i * 7) -= correction.segment<3>(i * 6);

        // Orientation correction from angular component
        Vector3d deltaTheta = correction.segment<3>(i * 6 + 3);
        Vector4d q = q_next.segment<4>(i * 7 + 3);

        // Apply small rotation using Omega matrix
        q_next.segment<4>(i * 7 + 3) = applySmallRotationQuaternion(q, deltaTheta);

      }

    }
  }

  // === 🔧 Velocity projection
  {
    MatrixXd J(m_numConstraints, dof_dq);
    VectorXd Jdq(m_numConstraints);
    int row = 0;
    for (const auto& c : m_constraints) {
      c->computeJacobian(J, row);
      Jdq.segment(row, c->getDOFs()) = J.block(row, 0, c->getDOFs(), dof_dq) * dq_next;
      row += c->getDOFs();
    }

    if (m_numConstraints > 0) {
      MatrixXd JJt = J * J.transpose();
      VectorXd correction = J.transpose() * JJt.completeOrthogonalDecomposition().solve(Jdq);
      dq_next -= correction;
    }
  }

  // === ✅ Final write-back
  for (int i = 0; i < m_numBodies; ++i) {
    auto& body = m_bodies[i];
    if (body->isFixed()) continue;

    body->setPosition(q_next.segment<3>(i * 7));

    Vector4d q = q_next.segment<4>(i * 7 + 3);
    q.normalize();
    body->setOrientation(q);

    body->setLinearVelocity(dq_next.segment<3>(i * 6));
    body->setAngularVelocity(dq_next.segment<3>(i * 6 + 3));
  }
}


UniqueID Dynamics::addBody(
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
    orientation
  ));
  m_bodyIndex.try_emplace(ID, m_bodies.size() - 1);
  m_numBodies++;
  return ID;
}

Body *Dynamics::getBody(const UniqueID ID) {
  auto it = m_bodyIndex.find(ID);
  if (it != m_bodyIndex.end()) {
    // Check if the index is valid
    if (it->second < m_bodies.size()) {
      return m_bodies[it->second].get();
    }
  }
  return nullptr; // Return nullptr if the body is not found
}

const Body *Dynamics::getBody(const UniqueID ID) const {
  auto it = m_bodyIndex.find(ID);
  if (it != m_bodyIndex.end()) {
    // Check if the index is valid
    if (it->second < m_bodies.size()) {
      return m_bodies[it->second].get();
    }
  }
  return nullptr; // Return nullptr if the body is not found
}

} // Proton
