// dynamics/ConstraintSolver.cpp
#include "ConstraintSolver.h"

#include <iostream>
#include <Eigen/LU>
#include "utils/Logger.h"

namespace Neutron {

ConstraintSolver::ConstraintSolver() = default;

void ConstraintSolver::addConstraint(std::shared_ptr<Constraint> constraint) {
  m_constraints.push_back(constraint);
}

void ConstraintSolver::clearConstraints() {
  m_constraints.clear();
}

void ConstraintSolver::buildJacobian(
    const std::vector<Particle*>& bodies,
    MatrixXd& jacobian,
    VectorXd& constraintRHS)
{
  // Create particle to index map
  std::map<Particle*, int> particleToIndex;
  for (int i = 0; i < bodies.size(); ++i) {
    particleToIndex[bodies[i]] = i;
  }

  // Collect qdot (velocities)
  VectorXd qdot(bodies.size() * 3);
  for (int i = 0; i < bodies.size(); ++i) {
    qdot.segment<3>(i * 3) = bodies[i]->getVelocity();
  }

  // Total number of constraints
  int numConstraints = 0;
  for (const auto& constraint : m_constraints) {
    numConstraints += constraint->getDOFs();
  }

  // Total number of generalized coordinates (3 DOF per particle)
  int numCoordinates = bodies.size() * 3;

  // Resize Jacobian and constraintRHS
  jacobian.resize(numConstraints, numCoordinates);
  jacobian.setZero();
  constraintRHS.resize(numConstraints);
  constraintRHS.setZero();

  // Current row in the Jacobian and constraintRHS
  int startRow = 0;
  for (const auto& constraint : m_constraints) {

    // Compute Jacobian with particle indices
    constraint->computeJacobian(jacobian, startRow, particleToIndex);

    // Compute C_dot = J_block * qdot
    MatrixXd J_block =
      jacobian.block(startRow, 0, constraint->getDOFs(), jacobian.cols());
    VectorXd C_dot =
      J_block * qdot;

    // Compute Jdot*qdot
    VectorXd gamma(constraint->getDOFs());
    gamma.setZero();
    constraint->computeJacobianDerivative(gamma, startRow);

    // Populate RHS:
    constraintRHS.segment(startRow, constraint->getDOFs()) =
      gamma;
    // gamma - (2 * m_alpha * C_dot) - ((m_beta * m_beta) * c)

    startRow += constraint->getDOFs();
  }

  LOG_DEBUG("Jacobian: \n", jacobian);
  LOG_DEBUG("RHS: \n", constraintRHS);

}

void ConstraintSolver::solveConstrainedSystem(
    const VectorXd& M,         // Mass matrix (diagonal)
    const VectorXd& forces,    // External forces
    const MatrixXd& jacobian,  // Constraint Jacobian
    const VectorXd& constraintRHS, // Constraint right-hand side
    VectorXd& accelerations,   // Output accelerations
    VectorXd& lambdas          // Output Lagrange multipliers
) {
  int n = M.size();       // Number of generalized coordinates
  int m = constraintRHS.size(); // Number of constraints

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

  LOG_DEBUG("Accelerations: \n", accelerations);
  LOG_DEBUG("Lambdas: \n", lambdas);
}

} // namespace neutron