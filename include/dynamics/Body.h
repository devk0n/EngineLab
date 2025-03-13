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
  [[nodiscard]] const double &getMass() const;
  [[nodiscard]] const Vector3d &getInertia() const;

  // State variables
  [[nodiscard]] const Vector3d &getPosition() const;
  [[nodiscard]] const Quaterniond &getOrientation() const;
  [[nodiscard]] const Vector3d &getVelocity() const;
  [[nodiscard]] const Vector3d &getAngularVelocity() const;

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
  [[nodiscard]] glm::vec3 getPositionVec3() const;
  [[nodiscard]] glm::quat getOrientationQuat() const;

  [[nodiscard]] double getMassInverse() const {
    return 1.0 / m_mass;
  }

  static Matrix3d skewSymmetric(Vector3d vector) {
    Matrix3d skew;
    skew <<           0, -vector.z(),  vector.y(),
             vector.z(),           0, -vector.x(),
            -vector.y(),  vector.x(),           0;
    return skew;
  }

  [[nodiscard]] Vector3d getGyroscopicTorque() const {
    Vector3d b = skewSymmetric(m_angularVelocity) * m_inertia.asDiagonal() *
                 m_angularVelocity;
    return b;
  }

  // Configuration
  void setFixed(bool fixed);
  bool isFixed() const;

  [[nodiscard]] Matrix3d getInverseInertiaTensor() const { return m_inertia.asDiagonal().inverse(); }

private:
  UniqueID m_ID;

  bool m_fixed = true;
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
