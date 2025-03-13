#ifndef SOLVER_H
#define SOLVER_H
#include "core/types.h"

namespace Neutron {
class Solver {
public:
  Solver() = default;

  void solveSystem(
      const VectorXd &M,
      const VectorXd &forces,
      VectorXd &accelerations
  ) const;
};
} // namespace Neutron

#endif // SOLVER_H
