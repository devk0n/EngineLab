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
    const VectorXd& M,                  // Mass matrix (diagonal) containing N and J
    const VectorXd& forces,             // Vector of forces for a system (g) with coriolis
    const MatrixXd& jacobian,           // Constraint Jacobian
    const VectorXd& gamma,              // Constraint right-hand side
    VectorXd& accelerations,            // Output accelerations
    VectorXd& lambdas
  ) const;

  void buildJacobian(
      MatrixXd &jacobian,
      VectorXd &gamma,
      const double &systemSize
  );

  void addConstraint(const std::shared_ptr<Constraint>& constraint) {
    m_constraints.push_back(constraint);
    m_numConstraints += constraint->getDOFs();
  }

  void clearConstraints() {
    m_constraints.clear();
    m_numConstraints = 0;
  }

private:
  std::vector<std::shared_ptr<Constraint>> m_constraints;
  int m_numConstraints = 0;

};
} // namespace Neutron

#endif // SOLVER_H
