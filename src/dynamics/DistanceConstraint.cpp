#include "DistanceConstraint.h"

namespace Proton {

DistanceConstraint::DistanceConstraint(
    Body* body1,
    Body* body2)
    : Constraint(1),
      m_body1(body1),
      m_body2(body2) {

  Vector3d d = m_body2->getPosition() - m_body1->getPosition();

  m_distance = d.norm();

}

void DistanceConstraint::computePositionError(VectorXd &phi, const int startRow) const {
  // Relative position
  Vector3d d = m_body2->getPosition() - m_body1->getPosition();

  // Constraint equation
  phi[startRow] = (d.transpose() * d) - (m_distance * m_distance);
}

void DistanceConstraint::computeJacobian(MatrixXd &jacobian, const int startRow) const {
  // Relative position
  Vector3d d = m_body2->getPosition() - m_body1->getPosition();

  // Jacobian matrix
  jacobian.block<1, 3>(startRow, m_body1->getIndex() * 3) = - 2 * d.transpose(); // Correct sign for particle 1
  jacobian.block<1, 3>(startRow, m_body2->getIndex() * 3) =   2 * d.transpose(); // Correct sign for particle 2
}

void DistanceConstraint::computeAccelerationCorrection(VectorXd &gamma, const int startRow) const {
  // Relative velocity
  Vector3d v = m_body2->getLinearVelocity() - m_body1->getLinearVelocity();

  const double result = 2.0 * v.squaredNorm();
  gamma[startRow] = result;
}
} // Proton
