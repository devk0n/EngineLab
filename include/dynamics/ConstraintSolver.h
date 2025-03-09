// dynamics/ConstraintSolver.h
#ifndef CONSTRAINT_SOLVER_H
#define CONSTRAINT_SOLVER_H

#include <memory>
#include <vector>
#include "Constraint.h"
#include "Particle.h"
#include "core/types.h"

namespace Neutron {

class ConstraintSolver {
public:
  ConstraintSolver();

  // Add constraints to the solver
  void addConstraint(const std::shared_ptr<Constraint>& constraint);
  void clearConstraints();

  // Build the constraint Jacobian matrix
  void buildJacobian(
      const std::vector<Particle*>& bodies,
      MatrixXd& jacobian,
      VectorXd& constraintRHS
  );

  // Solve the constrained system
  static void solveConstrainedSystem(
      const VectorXd& M,         // Mass matrix (diagonal)
      const VectorXd& forces,    // External forces
      const MatrixXd& jacobian,  // Constraint Jacobian
      const VectorXd& constraintRHS, // Constraint right-hand side
      VectorXd& accelerations,   // Output accelerations
      VectorXd& lambdas          // Output Lagrange multipliers
  );

private:
  std::vector<std::shared_ptr<Constraint>> m_constraints;
  double m_alpha = 200.0;
  double m_beta = 500.0;
};

} // namespace Neutron

#endif // CONSTRAINT_SOLVER_H