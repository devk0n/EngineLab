#ifndef BODY_H
#define BODY_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "core/types.h"

namespace Neutron {

class Body {
public:
  Body(
      UniqueID ID,
      double mass,
      const Vector3d &inertia,
      const Vector3d &position,
      const Quaterniond &orientation
  );

  UniqueID getID() const;

  // Physical properties
  double getMass() const;
  const Vector3d &getInertia() const;

  // State variables
  const Vector3d &getPosition() const;
  const Quaterniond &getOrientation() const;
  const Vector3d &getVelocity() const;
  const Vector3d &getAngularVelocity() const;

  void setPosition(const Vector3d &position);
  void setOrientation(const Quaterniond &orientation);
  void setVelocity(const Vector3d &velocity);
  void setAngularVelocity(const Vector3d &angularVelocity);

  // Force accumulation
  void addForce(const Vector3d &force);
  void clearForces();
  const Vector3d &getForce() const;

  void addTorque(const Vector3d &torque);
  void clearTorques();
  const Vector3d &getTorque() const;

  // Visualization convertion
  glm::vec3 getPositionVec3() const;
  glm::quat getOrientationQuat() const;

  static Matrix3d skewSymmetric(Vector3d vector) {
    Matrix3d skew;
    skew <<           0, -vector.z(),  vector.y(),
             vector.z(),           0, -vector.x(),
            -vector.y(),  vector.x(),           0;
    return skew;
  }

  static Eigen::Matrix<double, 3, 4> transformationG(Quaterniond e) {
    Eigen::Matrix<double, 3, 4> G;
    G << -e.x(),  e.w(), -e.z(),  e.y(),
         -e.y(),  e.z(),  e.w(), -e.x(),
         -e.z(), -e.y(),  e.x(),  e.w();
    return G;
  }

  static Eigen::Matrix<double, 3, 4> transformationL(Quaterniond e) {
    Eigen::Matrix<double, 3, 4> G;
    G << -e.x(),  e.w(),  e.z(), -e.y(),
         -e.y(), -e.z(),  e.w(),  e.x(),
         -e.z(),  e.y(), -e.x(),  e.w();
    return G;
  }

  [[nodiscard]] Matrix3d getRotationA() const {
    return m_orientation.toRotationMatrix();
  }

  [[nodiscard]] Vector3d getGyroscopicTorque() const {
    Vector3d b = skewSymmetric(m_angularVelocity) * m_inertia.asDiagonal() *
                 m_angularVelocity;
    return b;
  }

private:
  UniqueID m_ID;

  // Physical properties
  double m_mass;
  Vector3d m_inertia;

  // State variables
  Vector3d m_position;
  Quaterniond m_orientation;
  Vector3d m_velocity;
  Vector3d m_angularVelocity;

  // Force accumulators
  Vector3d m_force;
  Vector3d m_torque;
};

} // namespace Neutron

#endif // BODY_H
