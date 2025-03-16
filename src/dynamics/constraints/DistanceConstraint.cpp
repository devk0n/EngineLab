#include "DistanceConstraint.h"

namespace Neutron {
DistanceConstraint::DistanceConstraint(
    Body *body1,
    Body *body2,
    const double distance)
    : Constraint(1),
      m_body1(body1),
      m_body2(body2),
      m_distance(distance) {}

void DistanceConstraint::computePhi(VectorXd &phi, const int startRow) {

  Vector3d d = m_body2->getPosition() - m_body1->getPosition();

  phi[startRow] = (d.transpose() * d) - (m_distance * m_distance);
}

void DistanceConstraint::computeJacobian(MatrixXd &jacobian, const int startRow) {
  Vector3d d = m_body2->getPosition() - m_body1->getPosition();

  const int index1 = m_body1->getIndex();
  const int index2 = m_body2->getIndex();

  jacobian.block<1, 3>(startRow, index1 * 3) = -2 * d.transpose(); // Correct sign for particle 1
  jacobian.block<1, 3>(startRow, index2 * 3) =  2 * d.transpose(); // Correct sign for particle 2
}

void DistanceConstraint::computeGamma(VectorXd &gamma, const int startRow) {
  Vector3d v = m_body2->getVelocity() - m_body1->getVelocity();

  double result = v.transpose() * v; // Correct sign to positive
  gamma[startRow] = - 2 * result;
}

}

