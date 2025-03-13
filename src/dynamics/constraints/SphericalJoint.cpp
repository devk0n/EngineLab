#include "SphericalJoint.h"

namespace Neutron {

SphericalJoint::SphericalJoint(
    Body* body1,
    Body* body2,
    const Vector3d &vector1,
    const Vector3d &vector2)
      : Constraint(3),
        m_body1(body1),
        m_body2(body2),
        m_vector1(vector1),
        m_vector2(vector2) {}

void SphericalJoint::computeConstraintEquations(VectorXd &c, const int startRow) {

  auto r1 = m_body1->getPosition();
  auto r2 = m_body2->getPosition();

  auto A1 = LMatrix(m_body1->getOrientation()) * LMatrix(m_body1->getOrientation()).transpose();
  auto A2 = LMatrix(m_body2->getOrientation()) * LMatrix(m_body2->getOrientation()).transpose();

  auto s1Am = m_vector1;
  auto s2Am = m_vector2;

  auto Phi = r1 + A1 * s1Am - (r2 + A2 * s2Am);

  // Assemble Phi vector
  c.segment(startRow, m_DOFs) = Phi;
}

void SphericalJoint::computeJacobian(MatrixXd& jacobian, const int startRow, const std::map<Body*, int>& bodyToIndex) {

  auto A1 = LMatrix(m_body1->getOrientation()) * LMatrix(m_body1->getOrientation()).transpose();
  auto A2 = LMatrix(m_body2->getOrientation()) * LMatrix(m_body2->getOrientation()).transpose();

  auto s1Am = m_vector1;
  auto s2Am = m_vector2;

  auto I3 = Matrix3d::Identity();

  int index1 = bodyToIndex.at(m_body1);
  int index2 = bodyToIndex.at(m_body2);

  // Fill the Jacobian matrix
  jacobian.block<3, 3>(startRow, 6 * index1)     =  I3;               // Translational part of body 1
  jacobian.block<3, 3>(startRow, 6 * index1 + 3) = -A1 * skew(s1Am);    // Rotational part of body 1
  jacobian.block<3, 3>(startRow, 6 * index2)     = -I3;               // Translational part of body 2
  jacobian.block<3, 3>(startRow, 6 * index2 + 3) =  A2 * skew(s2Am);    // Rotational part of body 2

}

void SphericalJoint::computeJacobianDerivative(VectorXd &jdotqdot, int startRow) {
  // Get rotation matrices
  auto A1 = LMatrix(m_body1->getOrientation()) * LMatrix(m_body1->getOrientation()).transpose();
  auto A2 = LMatrix(m_body2->getOrientation()) * LMatrix(m_body2->getOrientation()).transpose();

  auto s1Am = m_vector1;
  auto s2Am = m_vector2;

  // Get angular velocities
  auto omega1 = m_body1->getAngularVelocity();
  auto omega2 = m_body2->getAngularVelocity();

  auto gamma = A1 * skew(omega1) * (skew(s1Am) * omega1) - A2 * skew(omega2) * (skew(s2Am) * omega2);

  // Assemble Phi vector
  jdotqdot.segment(startRow, m_DOFs) = gamma;
}




} // namespace Neutron