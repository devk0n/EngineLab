// dynamics/ConstraintSolver.h
#ifndef CONSTRAINT_SOLVER_H
#define CONSTRAINT_SOLVER_H

#include <memory>
#include <vector>
#include "Body.h"
#include "Constraint.h"
#include "core/types.h"

namespace Neutron {

class ConstraintSolver {
public:
  ConstraintSolver();

  // Add constraints to the solver
  void addConstraint(std::shared_ptr<Constraint> constraint);
  void clearConstraints();

  // Build the constraint Jacobian matrix
  void buildJacobian(
      const std::vector<Body*>& bodies,
      MatrixXd& jacobian,
      VectorXd& constraintRHS
  );

  // Solve the constrained system
  void solveSystem(
      const VectorXd& M,         // Mass matrix (diagonal)
      const VectorXd& forces,    // External forces and internal moments
      const MatrixXd& jacobian,  // Constraint Jacobian
      const VectorXd& constraintRHS, // Constraint right-hand side
      VectorXd& accelerations,   // Output accelerations
      VectorXd& lambdas          // Output Lagrange multipliers
  );

private:
  std::vector<std::shared_ptr<Constraint>> m_constraints;
};

} // namespace Neutron

#endif // CONSTRAINT_SOLVER_H