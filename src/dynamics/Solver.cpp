#include "Solver.h"

namespace Neutron {

void Solver::buildJacobian(
    MatrixXd &jacobian,
    VectorXd &gamma,
    const double &systemSize) {

  // Resize matrices to accommodate all constraints
  jacobian.resize(m_numConstraints, systemSize);
  gamma.resize(m_numConstraints);

  // Zero-initialize the matrices
  jacobian.setZero();
  gamma.setZero();

  // Have each constraint fill its portion of the matrices
  int rowOffset = 0;
  for (const auto& constraint : m_constraints) {
    constraint->computeJacobian(jacobian, rowOffset);
    constraint->computeGamma(gamma, rowOffset);
    rowOffset += constraint->getDOFs();
  }
}

void Solver::solveSystem(
  const VectorXd &M,        // Mass matrix (diagonal) containing N and J
  const VectorXd &forces,   // Vector of forces for a system (g) with coriolis
  const MatrixXd &jacobian, // Constraint Jacobian
  const VectorXd &gamma,    // Constraint right-hand side
  VectorXd &accelerations,  // Output accelerations
  VectorXd &lambdas         // Output Lagrange multipliers
) const {
  int m = static_cast<int>(m_constraints.size()); // Number of constraints

  if (m == 0) {
    // No constraints, direct solution
    accelerations = forces.array() / M.array();
    return;
  }

  // Step 1: Solve for Lagrange multipliers (λ)
  MatrixXd J_M_inv_JT = jacobian * M.asDiagonal().inverse() * jacobian.transpose();
  VectorXd rhs = gamma - jacobian * M.asDiagonal().inverse() * forces;
  lambdas = J_M_inv_JT.ldlt().solve(rhs);

  // Step 2: Solve for accelerations (q̈)
  accelerations = M.asDiagonal().inverse() * (forces + jacobian.transpose() * lambdas);
}
}
