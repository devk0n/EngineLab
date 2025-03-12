#include "SphericalJoint.h"

#include <iostream>

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
  // Constraint equation

  Vector3d r1 = m_body1->getPosition();
  Vector3d r2 = m_body2->getPosition();
  Matrix3d A1 = m_body1->getRotationA();
  Matrix3d A2 = m_body2->getRotationA();

  Vector3d s1Am = m_vector1;
  Vector3d s2Am = m_vector2;

  Vector3d constraint = r1 + A1 * s1Am - r2 - A2 * s2Am;

  // Assign each component to the corresponding row in 'c'
  c[startRow]     = constraint.x();
  c[startRow + 1] = constraint.y();
  c[startRow + 2] = constraint.z();
}

// Helper function to compute the skew-symmetric matrix of a vector
Matrix3d skewSymmetric(const Vector3d& v) {
  Matrix3d skew;
  skew <<  0,    -v.z(),  v.y(),
           v.z(),  0,    -v.x(),
          -v.y(),  v.x(),  0;
  return skew;
}

void SphericalJoint::computeJacobian(MatrixXd& jacobian, const int startRow, const std::map<Body*, int>& bodyToIndex) {

  auto A1 = m_body1->getRotationA();
  auto A2 = m_body2->getRotationA();

  auto s1Am = m_vector1;
  auto s2Am = m_vector2;

  auto I3 = Matrix3d::Identity();

  // Compute skew-symmetric matrices
  auto s1AmSkew = skewSymmetric(s1Am);
  auto s2AmSkew = skewSymmetric(s2Am);

  // Get the column indices for each body in the Jacobian
  int col1 = bodyToIndex.at(m_body1); // Start column for body 1
  int col2 = bodyToIndex.at(m_body2); // Start column for body 2

  // Fill the Jacobian matrix
  jacobian.block<3, 3>(startRow, col1)     =  I3;               // Translational part of body 1
  jacobian.block<3, 3>(startRow, col1 + 3) = -A1 * s1AmSkew;    // Rotational part of body 1
  jacobian.block<3, 3>(startRow, col2)     = -I3;               // Translational part of body 2
  jacobian.block<3, 3>(startRow, col2 + 3) =  A2 * s2AmSkew;    // Rotational part of body 2
}

void SphericalJoint::computeJacobianDerivative(VectorXd &jdotqdot, int startRow) {
  // Get rotation matrices
  Matrix3d A1 = m_body1->getRotationA();
  Matrix3d A2 = m_body2->getRotationA();

  // Get angular velocities
  Vector3d omega1 = m_body1->getAngularVelocity();
  Vector3d omega2 = m_body2->getAngularVelocity();

  // Compute rotated attachment points
  Vector3d s1Rot = A1 * m_vector1;
  Vector3d s2Rot = A2 * m_vector2;

  // Compute Ȧ = A * skew(ω) for each body
  Vector3d omega1_cross_s1 = omega1.cross(s1Rot);
  Vector3d omega2_cross_s2 = omega2.cross(s2Rot);

  // The complete J̇q̇ term for a spherical joint
  Vector3d gamma = omega1_cross_s1 - omega2_cross_s2;

  jdotqdot[startRow]     = gamma.x();
  jdotqdot[startRow + 1] = gamma.y();
  jdotqdot[startRow + 2] = gamma.z();
}




} // namespace Neutron