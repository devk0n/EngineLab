#include "Solver.h"

namespace Neutron {

void Solver::solveSystem(
  const VectorXd& M,
  const VectorXd& forces,
  const MatrixXd& jacobian,
  const MatrixXd& gamma,
  VectorXd& accelerations,
  VectorXd& lambdas) const {


  accelerations = forces.array() / M.array();


}

} // namespace Neutron

