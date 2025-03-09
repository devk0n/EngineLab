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
  void addConstraint(std::shared_ptr<Constraint> constraint);
  void clearConstraints();

  // Build the constraint Jacobian matrix
  void buildJacobian(
      const std::vector<Particle*>& bodies,
      MatrixXd& jacobian,
      VectorXd& constraintRHS,
      double alpha,
      double beta
  );

  // Solve the constrained system
  void solveConstrainedSystem(
      const VectorXd& M,         // Mass matrix (diagonal)
      const VectorXd& forces,    // External forces
      const MatrixXd& jacobian,  // Constraint Jacobian
      const VectorXd& constraintRHS, // Constraint right-hand side
      VectorXd& accelerations,   // Output accelerations
      VectorXd& lambdas          // Output Lagrange multipliers
  );

  // Solve method
  void solve(
      const VectorXd& M,         // Mass matrix (diagonal)
      const VectorXd& forces,    // External forces
      VectorXd& accelerations,   // Output accelerations
      VectorXd& lambdas,         // Output Lagrange multipliers
      const VectorXd& q,         // Current positions
      const VectorXd& qdot,      // Current velocities
      double alpha = 5.0,        // Baumgarte alpha parameter
      double beta = 5.0          // Baumgarte beta parameter
  );

private:
  std::vector<std::shared_ptr<Constraint>> m_constraints;
};

} // namespace Neutron

#endif // CONSTRAINT_SOLVER_H