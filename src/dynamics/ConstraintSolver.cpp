// dynamics/ConstraintSolver.cpp
#include "ConstraintSolver.h"

#include <Eigen/LU>
#include "utils/Logger.h"

// TODO: Rename/Refactor ConstraintSolver to Solver

namespace Neutron {

ConstraintSolver::ConstraintSolver() = default;

void ConstraintSolver::addConstraint(std::shared_ptr<Constraint> constraint) {
  m_constraints.push_back(constraint);
}

void ConstraintSolver::clearConstraints() {
  m_constraints.clear();
}

void ConstraintSolver::buildJacobian(
    const std::vector<Body*>& bodies,
    MatrixXd& jacobian,
    VectorXd& constraintRHS)
{
  // Create body to index map
  std::map<Body*, int> bodyToIndex;
  for (int i = 0; i < bodies.size(); ++i) {
    bodyToIndex[bodies[i]] = i;
  }

  // Collect qdot (velocities)
  VectorXd qdot(bodies.size() * 6);
  for (int i = 0; i < bodies.size(); ++i) {
    qdot.segment<3>(i * 6) = bodies[i]->getVelocity();
    qdot.segment<3>(i * 6 + 3) = bodies[i]->getAngularVelocity();
  }

  // Total number of constraints
  int numConstraints = 0;
  for (const auto& constraint : m_constraints) {
    numConstraints += constraint->getDOFs();
  }

  // Total number of generalized coordinates (6 DOF per body)
  int numCoordinates = bodies.size() * 6;

  // Resize Jacobian and constraintRHS
  jacobian.resize(numConstraints, numCoordinates);
  jacobian.setZero();
  constraintRHS.resize(numConstraints);
  constraintRHS.setZero();

  // Current row in the Jacobian and constraintRHS
  int startRow = 0;
  for (const auto& constraint : m_constraints) {

    // Compute Jacobian with particle indices
    constraint->computeJacobian(jacobian, startRow);

    // Compute C_dot = J_block * qdot
    MatrixXd J_block = jacobian.block(startRow, 0, constraint->getDOFs(), jacobian.cols());
    VectorXd C_dot = J_block * qdot;

    // Compute Jdot*qdot
    VectorXd gamma(constraint->getDOFs());
    gamma.setZero();
    constraint->computeGamma(gamma, startRow);

    // Populate RHS:
    constraintRHS.segment(startRow, constraint->getDOFs()) = gamma;

    startRow += constraint->getDOFs();
  }
}

void ConstraintSolver::solveSystem(
    const VectorXd& M,                  // Mass matrix (diagonal) containing N and J
    const VectorXd& forces,             // Vector of forces for a system (g) with coriolis
    const MatrixXd& jacobian,           // Constraint Jacobian
    const VectorXd& constraintRHS,      // Constraint right-hand side
    VectorXd& accelerations,            // Output accelerations
    VectorXd& lambdas                   // Output Lagrange multipliers
) {
  int n = static_cast<int>(M.size());             // Number of generalized coordinates
  int m = static_cast<int>(constraintRHS.size()); // Number of constraints

  if (m == 0) {
    // No constraints, direct solution
    accelerations = forces.array() / M.array();
    lambdas.resize(0);
    return;
  }

  // Step 1: Solve for Lagrange multipliers (λ)
  MatrixXd J_M_inv_JT = jacobian * M.asDiagonal().inverse() * jacobian.transpose();
  VectorXd rhs = constraintRHS - jacobian * M.asDiagonal().inverse() * forces;
  lambdas = J_M_inv_JT.ldlt().solve(rhs);

  // Step 2: Solve for accelerations (q̈)
  accelerations = M.asDiagonal().inverse() * (forces + jacobian.transpose() * lambdas);
}

} // namespace Neutron