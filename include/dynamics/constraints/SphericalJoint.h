#ifndef SPHERICAL_JOINT_H
#define SPHERICAL_JOINT_H

#include "dynamics/Constraint.h"

namespace Neutron {
class SphericalJoint : public Constraint {
public:
  SphericalJoint(
      Body* body1,
      Body* body2,
      const Vector3d &vector1,
      const Vector3d &vector2
  );

  void computePhi(VectorXd& c, int startRow) override;
  void computeJacobian(MatrixXd &jacobian, int startRow) override;
  void computeGamma(VectorXd &jdotqdot, int startRow) override;

private:
  Body* m_body1;
  Body* m_body2;
  Vector3d m_vector1;
  Vector3d m_vector2;
};
} // namespace Neutron

#endif // SPHERICAL_JOINT_H
