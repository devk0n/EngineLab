#include "SphericalJoint.h"

namespace Neutron {

SphericalJoint::SphericalJoint(
    Body* body1,
    Body* body2,
    const Vector3d& vector1,
    const Vector3d& vector2)
    : Constraint(3),
      m_body1(body1),
      m_body2(body2),
      m_vector1(vector1),
      m_vector2(vector2) {}

void SphericalJoint::computePhi(VectorXd &phi, const int startRow) {
  auto r1 = m_body1->getPosition();
  auto r2 = m_body2->getPosition();

  auto A1 = LMatrix(m_body1->getOrientation()) * LMatrix(m_body1->getOrientation()).transpose();
  auto A2 = LMatrix(m_body2->getOrientation()) * LMatrix(m_body2->getOrientation()).transpose();

  auto s1Am = m_vector1;
  auto s2Am = m_vector2;

  // Assemble Phi vector
  phi.segment(startRow, m_DOFs) = r1 + A1 * s1Am - (r2 + A2 * s2Am);
}

void SphericalJoint::computeJacobian(MatrixXd& jacobian, const int startRow) {
  auto A1 = LMatrix(m_body1->getOrientation()) * LMatrix(m_body1->getOrientation()).transpose();
  auto A2 = LMatrix(m_body2->getOrientation()) * LMatrix(m_body2->getOrientation()).transpose();

  auto s1Am = m_vector1;
  auto s2Am = m_vector2;

  auto I3 = Matrix3d::Identity();

  int index1 = 0;
  int index2 = 1;

  // Fill the Jacobian matrix
  jacobian.block<3, 3>(startRow, 6 * index1)     =  I3;                 // Translational part of body 1
  jacobian.block<3, 3>(startRow, 6 * index1 + 3) = -skew(s1Am) * A1;    // Rotational part of body 1
  jacobian.block<3, 3>(startRow, 6 * index2)     = -I3;                 // Translational part of body 2
  jacobian.block<3, 3>(startRow, 6 * index2 + 3) =  skew(s2Am) * A2;    // Rotational part of body 2
}

void SphericalJoint::computeGamma(VectorXd& gamma, const int startRow) {
  auto A1 = LMatrix(m_body1->getOrientation()) * LMatrix(m_body1->getOrientation()).transpose();
  auto A2 = LMatrix(m_body2->getOrientation()) * LMatrix(m_body2->getOrientation()).transpose();

  auto s1Am = m_vector1;
  auto s2Am = m_vector2;

  // Get angular velocities
  auto omega1 = m_body1->getAngularVelocity();
  auto omega2 = m_body2->getAngularVelocity();

  auto s1Ad = A1 * skew(omega1) * s1Am;
  auto s2Ad = A2 * skew(omega2) * s2Am;

  // Assemble Phi vector
  gamma.segment(startRow, m_DOFs) = - skew(omega1) * s1Ad + skew(omega2) * s2Ad;
}

} // namespace Neutron


