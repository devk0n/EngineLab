#ifndef BODY_H
#define BODY_H

#include "Proton.h"
#include "glm/glm.hpp"
#include "glm/detail/type_quat.hpp"

namespace Proton {
class Body {
public:
  Body(
      const UniqueID ID,
      const int index,
      const double mass,
      const Vector3d &inertia,
      const Vector3d &position,
      const Vector4d &orientation)
      : m_ID(ID),
        m_index(index),
        m_mass(mass),
        m_inertia(inertia),
        m_position(position),
        m_orientation(orientation) {
    m_inverseMass = 1.0 / mass;
    m_inverseInertia = Vector3d(1.0 / inertia.x(), 1.0 / inertia.y(), 1.0 / inertia.z());
  }

  // Update state
  void updateInertiaWorld() {
    // Convert quaternion (w, x, y, z) to rotation matrix
    const double w = m_orientation(0);
    const double x = m_orientation(1);
    const double y = m_orientation(2);
    const double z = m_orientation(3);

    const double ww = w * w;
    const double xx = x * x;
    const double yy = y * y;
    const double zz = z * z;

    const double wx = w * x, wy = w * y, wz = w * z;
    const double xy = x * y, xz = x * z;
    const double yz = y * z;

    Matrix3d R;
    R << ww + xx - yy - zz,       2 * (xy - wz),       2 * (xz + wy),
         2 * (xy + wz),       ww - xx + yy - zz,       2 * (yz - wx),
         2 * (xz - wy),           2 * (yz + wx),   ww - xx - yy + zz;

    // Transform inertia to world frame
    m_inverseInertiaWorld = R * m_inverseInertia.asDiagonal() * R.transpose();
  }

  // Getters for state
  UniqueID getID() const { return m_ID; }
  int getIndex() const { return m_index; }

  Vector3d getPosition() const { return m_position; }
  Vector3d getLinearVelocity() const { return m_velocity; }

  Vector4d getOrientation() const { return m_orientation; }
  Vector3d getAngularVelocity() const { return m_angularVelocity; }

  double getMass() const { return m_mass; }
  double getInverseMass() const { return m_inverseMass; }

  Matrix3d getInertiaWorld() const { return m_inverseInertiaWorld; }

  // Setters for state
  void setPosition(const Vector3d &position) { m_position = position; }
  void setLinearVelocity(const Vector3d &velocity) { m_velocity = velocity; }

  void setOrientation(const Vector4d &orientation) { m_orientation = orientation; }
  void setAngularVelocity(const Vector3d &angularVelocity) { m_angularVelocity = angularVelocity; }

  // Force accumulation
  void addForce(const Vector3d& force) { m_force.noalias() += force; }
  void clearForces() { m_force.setZero(); }
  const Vector3d& getForce() const { return m_force; }

  // Torque accumulation
  void addTorque(const Vector3d& torque) { m_torque.noalias() += torque; }
  void clearTorque() { m_torque.setZero(); }
  const Vector3d& getTorque() const { return m_torque; }

  // Configuration
  void setFixed(const bool fixed) { m_fixed = fixed; }
  bool isFixed() const { return m_fixed; }

  // Visualization convertion
  [[nodiscard]] glm::vec3 getPositionVec3() const {
    return {m_position.x(), m_position.y(), m_position.z()};
  }

  [[nodiscard]] glm::quat getOrientationQuat() const {
    return {
      // Cast w to float
      static_cast<float>(m_orientation.x()), // Cast x to float
      static_cast<float>(m_orientation.y()), // Cast y to float
      static_cast<float>(m_orientation.z()),  // Cast z to float
      static_cast<float>(m_orientation.w())
  };
  }

private:
  UniqueID m_ID = -1;
  int m_index = -1;

  // Physical properties
  double m_mass;
  double m_inverseMass;
  Vector3d m_inertia;
  Vector3d m_inverseInertia;
  Matrix3d m_inverseInertiaWorld;

  // State variables
  Vector3d m_position{Vector3d::Zero()};
  Vector3d m_velocity{Vector3d::Zero()};
  Vector4d m_orientation{Vector4d(1, 0, 0, 0)};
  Vector3d m_angularVelocity{Vector3d::Zero()};

  // Force accumulators
  Vector3d m_force{Vector3d::Zero()};
  Vector3d m_torque{Vector3d::Zero()};

  bool m_fixed = false;

};
} // Proton

#endif // BODY_H
