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
      const Vector4d &orientation)
      : m_ID(ID),
        m_mass(mass),
        m_inertia(inertia),
        m_position(position),
        m_orientation(orientation) {}

  UniqueID getID();

  // Force accumulation
  [[nodiscard]] const Vector3d& getForce() const { return m_force; }
  void addForce(const Vector3d& force) { m_force += force; }
  void clearForces() { m_force = Vector3d::Zero(); }

  // Torque accumulation
  [[nodiscard]] const Vector3d& getTorque() const { return m_torque; }
  void addTorque(const Vector3d& torque) { m_torque += torque; }
  void clearTorques() { m_torque = Vector3d::Zero(); }

  // State variables
  [[nodiscard]] const double& getMass() const { return m_mass; }
  [[nodiscard]] const Vector3d& getInertia() const { return m_inertia; }
  [[nodiscard]] const Vector3d& getPosition() const { return m_position; }
  [[nodiscard]] const Vector4d& getOrientation() const { return m_orientation; }
  [[nodiscard]] const Vector3d& getVelocity() const { return m_velocity; }
  [[nodiscard]] const Vector3d& getAngularVelocity() const { return m_angularVelocity; }

  void setPosition(const Vector3d& position) { m_position = position;}
  void setOrientation(const Vector4d& orientation) { m_orientation = orientation;}
  void setVelocity(const Vector3d& velocity) { m_velocity = velocity;}
  void setAngularVelocity(const Vector3d& angularVelocity) { m_angularVelocity = angularVelocity;}

  // Visualization convertion
  glm::vec3 getPositionVec3() const {
    return {m_position.x(), m_position.y(), m_position.z()};
  }

  glm::quat getOrientationQuat() const {
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

  // Physical properties
  double m_mass = 3;
  Vector3d m_inertia{Vector3d::Ones()};

  // State variables
  Vector3d m_position{Vector3d::Zero()};
  Vector4d m_orientation{Vector4d(1, 0, 0, 0)};
  Vector3d m_velocity{Vector3d::Zero()};
  Vector3d m_angularVelocity{Vector3d::Zero()};

  // Force accumulators
  Vector3d m_force{Vector3d::Zero()};
  Vector3d m_torque{Vector3d::Zero()};

};

} // namespace Neutron

#endif // BODY_H
