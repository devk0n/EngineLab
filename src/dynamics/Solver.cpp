#include "Solver.h"

namespace Neutron {

void Solver::solveSystem(
    const VectorXd& M,
    const VectorXd& forces,
    const VectorXd& phi,
    const MatrixXd& jacobian,
    const MatrixXd& gamma,
    VectorXd& accelerations,
    VectorXd& lambdas
) const {
  int m = static_cast<int>(lambdas.size()); // Number of constraints

  if (m == 0) {
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

} // namespace Neutron

