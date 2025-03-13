#include "DistanceConstraint.h"

namespace Neutron {

DistanceConstraint::DistanceConstraint(
    Body *body1,
    Body *body2)
    : Constraint(1),
      m_body1(body1),
      m_body2(body2) {

  m_distance = (m_body2->getPosition() - m_body1->getPosition()).norm();

  m_index1 = static_cast<int>(body1->getID());
  m_index2 = static_cast<int>(body2->getID());

}

DistanceConstraint::DistanceConstraint(
    Body *body1,
    Body *body2,
    const double distance)
    : Constraint(1),
      m_body1(body1),
      m_body2(body2),
      m_distance(distance) {

  m_index1 = static_cast<int>(body1->getID());
  m_index2 = static_cast<int>(body2->getID());

}

void DistanceConstraint::computePhi(VectorXd &phi, const int startRow) {
  Vector3d d = m_body2->getPosition() - m_body1->getPosition();
  phi[startRow] = d.squaredNorm() - (m_distance * m_distance);
}

void DistanceConstraint::computeJacobian(MatrixXd &jacobian, const int startRow) {
  Vector3d d = m_body2->getPosition() - m_body1->getPosition();
  jacobian.block<1, 3>(startRow, 6 * m_index1) = -2 * d.transpose();
  jacobian.block<1, 3>(startRow, 6 * m_index2) = 2 * d.transpose();
}

void DistanceConstraint::computeGamma(
    VectorXd &gamma,
    const int startRow) {
  Vector3d d = m_body2->getVelocity() - m_body1->getVelocity();
  const double result = - 2.0 * d.transpose() * d;
  gamma[startRow] = result;

}

}