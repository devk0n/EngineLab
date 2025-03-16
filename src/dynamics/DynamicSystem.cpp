#include "DynamicSystem.h"

namespace Neutron {
DynamicSystem::DynamicSystem() = default;

void DynamicSystem::buildMassTensor() {
  if (!m_dirty) return;

  int n = static_cast<int>(m_bodies.size()) * 3;
  m_massInertiaTensor.resize(n);
  m_massInertiaTensor.setZero();

  int i = 0;
  for (const auto &body: m_bodies) {
    m_massInertiaTensor.segment<3>(i * 3).setConstant(body->getMass());
    ++i;
  }

  m_dirty = false;
}

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

void DynamicSystem::step(double dt) {
  // Choose a max step size, e.g. 20ms
  constexpr double maxStep = 0.005;

  // If dt is too large, clamp it and warn
  if (dt > maxStep) {
    LOG_WARN("DeltaTime (", dt, ") exceeded max step (", maxStep, "). Clamping.");

    dt = maxStep;
  }

  if (m_bodies.empty()) {
    return;
  }

  // 1) Build mass data if needed
  buildMassTensor(); // ensures m_massInertiaTensor is up-to-date

  //------------------------------------------------------------
  // 2) Identify which bodies are free (not fixed)
  //------------------------------------------------------------
  // We'll create a vector of (bodyIndex -> solverIndex).
  // If body is fixed, we store -1. If free, we store a valid index 0..(numFreeBodies-1).
  std::vector<int> solverIndexOfBody(m_bodies.size(), -1);
  int numFreeBodies = 0;
  for (size_t i = 0; i < m_bodies.size(); ++i) {
    if (!m_bodies[i]->isFixed()) {
      solverIndexOfBody[i] = numFreeBodies;
      numFreeBodies++;
    }
  }
  if (numFreeBodies == 0) {
    // Everything is fixed; no need to integrate
    return;
  }

  const int dim = 3 * numFreeBodies; // translational DOFs for free bodies only

  //------------------------------------------------------------
  // 3) Build the (dim x dim) mass matrix for the free bodies
  //------------------------------------------------------------
  MatrixXd M = MatrixXd::Zero(dim, dim);
  for (size_t i = 0; i < m_bodies.size(); ++i) {
    int si = solverIndexOfBody[i];
    if (si < 0) {
      // body is fixed; skip
      continue;
    }
    // Fill the 3x3 block
    // m_massInertiaTensor(3*i) is mass for body i
    double mVal = m_massInertiaTensor(3 * i);
    M.block<3, 3>(3 * si, 3 * si) = mVal * Matrix3d::Identity();
  }

  //------------------------------------------------------------
  // 4) Gather the system state (positions x_n, velocities v_n)
  //    only for free bodies
  //------------------------------------------------------------
  VectorXd x_n(dim), v_n(dim); {
    int freeCounter = 0;
    for (size_t i = 0; i < m_bodies.size(); ++i) {
      int si = solverIndexOfBody[i];
      if (si < 0) {
        continue; // fixed
      }
      x_n.segment<3>(3 * si) = m_bodies[i]->getPosition();
      v_n.segment<3>(3 * si) = m_bodies[i]->getVelocity();
    }
  }

  // Initial guesses
  VectorXd xGuess = x_n;
  VectorXd vGuess = v_n;

  //------------------------------------------------------------
  // 5) Count total constraint DOFs: C
  //------------------------------------------------------------
  // If you have M constraints, each can contribute 1..3 DOFs, etc.
  const int C = m_numConstraints;

  VectorXd lambdaGuess = VectorXd::Zero(C);

  // Newton iteration parameters
  constexpr int maxNewtonIters = 3;
  constexpr double tolerance = 1e-6;

  //------------------------------------------------------------
  // 6) Newton iteration
  //------------------------------------------------------------
  for (int iter = 0; iter < maxNewtonIters; ++iter) {
    // 6a) Midpoint
    VectorXd xMid = 0.5 * (x_n + xGuess);
    VectorXd vMid = 0.5 * (v_n + vGuess);

    // 6b) Build residual: [resPos; resVel; resCon]
    VectorXd RES(2 * dim + C);
    RES.setZero();

    // --- resPos = xGuess - x_n - dt * vMid
    VectorXd resPos = xGuess - x_n - dt * vMid;
    RES.segment(0, dim) = resPos;

    // --- partial resVel = M*(vGuess - v_n) - dt * Fext
    VectorXd resVel = M * (vGuess - v_n);

    // External force = gravity for free bodies
    // (the fixed bodies do not appear in the system)
    for (size_t i = 0; i < m_bodies.size(); ++i) {
      int si = solverIndexOfBody[i];
      if (si < 0) {
        continue; // skip fixed
      }
      double mVal = m_massInertiaTensor(3 * i);
      // Suppose gravity is (0,0,-9.81)
      Vector3d Fg = mVal * Vector3d(0, 0, -9.81);
      resVel.segment<3>(3 * si) -= dt * Fg;
    }

    // We'll add the + dt * J^T * lambda part after we assemble J_mid below

    // (iii) Constraint residual
    VectorXd resCon(C);
    resCon.setZero();

    // We'll need J_mid of size (C x dim)
    MatrixXd J_mid(C, dim);
    J_mid.setZero();

    // 6c) Evaluate each constraint at the midpoint
    {
      int rowOffset = 0;
      for (auto &constraint: m_constraints) {
        int cDim = constraint->getDOFs(); // 1 for a distance constraint, etc.

        // We do a “fake positions” approach to evaluate Phi(xMid).
        // First, store old positions:
        std::vector<Vector3d> oldPos(m_bodies.size());
        for (size_t b = 0; b < m_bodies.size(); ++b) {
          oldPos[b] = m_bodies[b]->getPosition();
        }

        // Overwrite with midpoint positions for bodies that are free
        for (size_t b = 0; b < m_bodies.size(); ++b) {
          if (solverIndexOfBody[b] < 0) {
            // fixed body => do nothing, it is effectively an anchor
            // or keep its real position if you prefer
            continue;
          }
          // set midpoint
          int si = solverIndexOfBody[b];
          m_bodies[b]->setPosition(xMid.segment<3>(3 * si));
        }

        // Now gather constraint’s Phi and Jacobian
        VectorXd phiLocal(cDim);
        phiLocal.setZero();

        // The constraint’s `computePhi(...)` typically writes into the big array.
        // We'll just pass startRow=0 and store into a local array, then copy out.
        constraint->computePhi(phiLocal, 0);

        // The constraint’s `computeJacobian(...)` writes into a local cDim x (3*N).
        // But now we have fewer DOFs for free bodies. We can do one of two:
        //   (A) Let the constraint still fill a  cDim x (3*N) matrix,
        //       then we copy the columns for the free bodies into J_mid.
        //   (B) Modify the constraint so it only references free bodies directly.
        //
        // For demonstration, we assume (A) is used:
        MatrixXd jacLocal(cDim, 3 * m_bodies.size());
        jacLocal.setZero();
        constraint->computeJacobian(jacLocal, 0);

        // Copy phiLocal into resCon
        resCon.segment(rowOffset, cDim) = phiLocal;

        // Map from big jacLocal (C x 3*N) to the smaller J_mid (C x dim)
        // For each body b:
        for (size_t b = 0; b < m_bodies.size(); ++b) {
          int si = solverIndexOfBody[b];
          if (si < 0) {
            // fixed => skip
            continue;
          }
          // Copy the 3 columns from b into the 3 columns for si
          J_mid.block(rowOffset, 3 * si, cDim, 3)
              = jacLocal.block(0, 3 * b, cDim, 3);
        }

        // Restore old positions
        for (size_t b = 0; b < m_bodies.size(); ++b) {
          m_bodies[b]->setPosition(oldPos[b]);
        }

        rowOffset += cDim;
      } // end constraints
    }

    // Insert resCon into RES
    RES.segment(2 * dim, C) = resCon;

    // The constraint force part: + dt * J_mid^T * lambda
    resVel += dt * (J_mid.transpose() * lambdaGuess);

    // Put resVel into RES
    RES.segment(dim, dim) = resVel;

    // Check convergence
    double resNorm = RES.norm();
    if (resNorm < tolerance) {
      break;
    }

    //------------------------------------------------------------
    // 6d) Build the Jacobian (2*dim + C) x (2*dim + C)
    //------------------------------------------------------------
    MatrixXd JAC(2 * dim + C, 2 * dim + C);
    JAC.setZero();

    // dResPos/dX = I
    JAC.block(0, 0, dim, dim).setIdentity();
    // dResPos/dV = -(dt/2)*I
    JAC.block(0, dim, dim, dim) = -(dt / 2.0) * MatrixXd::Identity(dim, dim);

    // dResVel/dV = M
    JAC.block(dim, dim, dim, dim) = M;

    // dResVel/dLambda = dt * J_mid^T
    JAC.block(dim, 2 * dim, dim, C) = dt * J_mid.transpose();

    // dResCon/dX = J_mid
    JAC.block(2 * dim, 0, C, dim) = J_mid;

    // Solve for deltaX = [delta_x; delta_v; delta_lambda]
    VectorXd rhs = -RES;
    VectorXd deltaX = JAC.fullPivLu().solve(rhs);

    // Unpack
    VectorXd delta_x = deltaX.segment(0, dim);
    VectorXd delta_v = deltaX.segment(dim, dim);
    VectorXd delta_lambda = deltaX.segment(2 * dim, C);

    // Update guesses
    xGuess += delta_x;
    vGuess += delta_v;
    lambdaGuess += delta_lambda;
  } // end Newton iteration

  //------------------------------------------------------------
  // 7) Write back final solutions to the free bodies only
  //------------------------------------------------------------
  for (size_t i = 0; i < m_bodies.size(); ++i) {
    int si = solverIndexOfBody[i];
    if (si < 0) {
      // fixed, do not overwrite
      continue;
    }
    m_bodies[i]->setPosition(xGuess.segment<3>(3 * si));
    m_bodies[i]->setVelocity(vGuess.segment<3>(3 * si));
  }
}

void DynamicSystem::getVelocityState(VectorXd &velocities) const {
  velocities.resize(m_bodies.size() * 3);
  for (size_t i = 0; i < m_bodies.size(); ++i) {
    velocities.segment<3>(i * 3) = m_bodies[i]->getVelocity();
  }
}

void DynamicSystem::setVelocityState(const VectorXd &velocities) const {
  for (size_t i = 0; i < m_bodies.size(); ++i) {
    m_bodies[i]->setVelocity(velocities.segment<3>(i * 3));
  }
}

void DynamicSystem::setSystemState(VectorXd state) {
  int i = 0;
  for (auto &body: m_bodies) {
    body->setPosition(state.segment<3>(i * 3));
    ++i;
  }
}

void DynamicSystem::addConstraint(
  const std::shared_ptr<Constraint> &constraint) {
  m_constraints.emplace_back(constraint);
  m_numConstraints += constraint->getDOFs();
}

void DynamicSystem::getSystemState(VectorXd &state) const {
  state.resize(m_numBodies * 3);

  int i = 0;
  for (const auto &body: m_bodies) {
    state.segment<3>(i * 3) = body->getPosition();
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
} // namespace Neutron
