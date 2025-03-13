#include "DistanceConstraint.h"

namespace Neutron {

DistanceConstraint::DistanceConstraint(
    Body *body1,
    Body *body2)
    : Constraint(1),
      m_body1(body1),
      m_body2(body2) {

  m_index1 = static_cast<int>(body1->getID());
  m_index2 = static_cast<int>(body2->getID());

  // Compute distance
  Vector3d d = m_body2->getPosition() - m_body1->getPosition();
  m_distance = d.norm();

}

void DistanceConstraint::computePhi(
    VectorXd &phi,
    const int startRow) {
  // Relative position
  Vector3d d = m_body2->getPosition() - m_body1->getPosition();

  // Constraint equation
  phi[startRow] = d.transpose() * d - (m_distance * m_distance);

}

void DistanceConstraint::computeJacobian(
    MatrixXd &jacobian,
    const int startRow) {

  Vector3d d = m_body2->getPosition() - m_body1->getPosition();

  // Fill the Jacobian matrix
  jacobian.block<1, 3>(startRow, 6 * m_index1) = -2 * d.transpose();               // Translational part of body 1
  jacobian.block<1, 3>(startRow, 6 * m_index2) = 2 * d.transpose();               // Translational part of body 2

}

void DistanceConstraint::computeGamma(
    VectorXd &gamma,
    const int startRow) {
  Vector3d d = m_body2->getVelocity() - m_body1->getVelocity();
  const double result = - 2.0 * d.transpose() * d; // Correct sign to positive
  gamma[startRow] = result;

}

}