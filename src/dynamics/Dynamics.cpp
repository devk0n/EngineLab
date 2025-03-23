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
    velocityState.segment<3>(i * 3) = body->getVelocity();
    ++i;
  }

  // LOG_DEBUG("Velocity State: \n", velocityState);

  return velocityState;
}

void Dynamics::step(double dt) {
  const int dof = m_numBodies * 3;

  // Gather current state
  VectorXd q_n(dof);
  VectorXd dq_n(dof);
  for (int i = 0; i < m_numBodies; ++i) {
    q_n.segment<3>(i * 3) = m_bodies[i]->getPosition();
    dq_n.segment<3>(i * 3) = m_bodies[i]->getVelocity();
  }

  // Initial guess
  VectorXd q_next = q_n + dt * dq_n;
  VectorXd dq_next = dq_n;

  constexpr int maxIters = 10;
  constexpr double tol = 1e-8;

  for (int iter = 0; iter < maxIters; ++iter) {
    VectorXd q_mid = 0.5 * (q_n + q_next);
    VectorXd dq_mid = 0.5 * (dq_n + dq_next);

    // Update bodies to midpoint state
    for (int i = 0; i < m_numBodies; ++i) {
      m_bodies[i]->clearForces();
      if (m_bodies[i]->isFixed()) continue;
      m_bodies[i]->setPosition(q_mid.segment<3>(i * 3));
      m_bodies[i]->setVelocity(dq_mid.segment<3>(i * 3));
    }

    // Apply force generators
    for (const auto& fg : m_forceGenerators) {
      fg->apply(dt);
    }

    // Collect external forces
    VectorXd F_ext = VectorXd::Zero(dof);
    for (int i = 0; i < m_numBodies; ++i) {
      F_ext.segment<3>(i * 3) = m_bodies[i]->getForce();
    }

    // Build mass matrix (diagonal)
    MatrixXd M = MatrixXd::Zero(dof, dof);
    for (int i = 0; i < m_numBodies; ++i) {
      double m = m_bodies[i]->getMass();
      if (m > 0.0) {
        M.block<3, 3>(i * 3, i * 3) = m * Matrix3d::Identity();
      }
    }

    MatrixXd P = MatrixXd::Zero(m_numConstraints, dof);
    VectorXd gamma = VectorXd::Zero(m_numConstraints);

    int row = 0;
    for (const auto& c : m_constraints) {
      c->computeJacobian(P, row);
      c->computeAccelerationCorrection(gamma, row);
      row += c->getDOFs();
    }

    // Build KKT system
    MatrixXd KKT(dof + m_numConstraints, dof + m_numConstraints);
    KKT.setZero();
    KKT.topLeftCorner(dof, dof) = M;
    KKT.topRightCorner(dof, m_numConstraints) = P.transpose();
    KKT.bottomLeftCorner(m_numConstraints, dof) = P;

    VectorXd rhs(dof + m_numConstraints);
    rhs.head(dof) = F_ext;
    rhs.tail(m_numConstraints) = -gamma;

    VectorXd sol = KKT.ldlt().solve(rhs);
    VectorXd ddq_mid = sol.head(dof);

    // Midpoint integration
    VectorXd dq_new = dq_n + dt * ddq_mid;
    VectorXd q_new = q_n + dt * 0.5 * (dq_n + dq_new);

    // Check for convergence
    double err = (dq_new - dq_next).norm() + (q_new - q_next).norm();
    dq_next = dq_new;
    q_next = q_new;
    if (err < tol) break;
  }

  // === 🔧 Position projection using orthogonal projection formula
  {
    VectorXd phi(m_numConstraints);
    MatrixXd J(m_numConstraints, dof);

    int row = 0;
    for (const auto& c : m_constraints) {
      c->computePositionError(phi, row);           // Position-level constraint violation
      c->computeJacobian(J, row);        // Jacobian w.r.t. position
      row += c->getDOFs();
    }

    if (m_numConstraints > 0) {
      MatrixXd JJt = J * J.transpose();
      VectorXd correction = J.transpose() * JJt.ldlt().solve(phi);
      q_next -= correction;
    }
  }

  // === 🔧 Velocity projection using orthogonal projection formula
  {
    MatrixXd J(m_numConstraints, dof);
    VectorXd Jdq(m_numConstraints);

    int row = 0;
    for (const auto& c : m_constraints) {
      c->computeJacobian(J, row);
      Jdq.segment(row, c->getDOFs()) = J.block(row, 0, c->getDOFs(), dof) * dq_next;
      row += c->getDOFs();
    }

    if (m_numConstraints > 0) {
      MatrixXd JJt = J * J.transpose();
      VectorXd correction = J.transpose() * JJt.ldlt().solve(Jdq);
      dq_next -= correction;
    }
  }

  // === ✅ Final write-back to bodies
  for (int i = 0; i < m_numBodies; ++i) {
    if (m_bodies[i]->isFixed()) continue;
    m_bodies[i]->setPosition(q_next.segment<3>(i * 3));
    m_bodies[i]->setVelocity(dq_next.segment<3>(i * 3));
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
