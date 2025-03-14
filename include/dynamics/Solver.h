#ifndef SOLVER_H
#define SOLVER_H
#include <Constraint.h>
#include <memory>

#include "core/types.h"

namespace Neutron {
class Solver {
public:
  Solver() = default;

  void solveSystem(
    const VectorXd &M,
    const VectorXd &forces,
    const MatrixXd& jacobian,
    const MatrixXd& gamma,
    VectorXd &accelerations,
    VectorXd& lambdas
  ) const;

};
} // namespace Neutron

#endif // SOLVER_H
