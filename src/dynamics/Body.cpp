#include "Body.h"

namespace Neutron {

Body::Body(
  const UniqueID ID,
  const double mass,
  const Vector3d &inertia,
  const Vector3d &position,
  const Quaterniond &orientation)
    : m_ID(ID),
      m_mass(mass),
      m_inertia(inertia),
      m_position(position),
      m_orientation(orientation),
      m_velocity(Vector3d::Zero()),
      m_angularVelocity(Vector3d::Zero()),
      m_force(Vector3d::Zero()),
      m_torque(Vector3d::Zero()) {}

UniqueID Body::getID() const { return m_ID; }

// Physical properties
const double &Body::getMass() const { return m_mass; }
const Vector3d &Body::getInertia() const { return m_inertia; }

// State variables
const Vector3d &Body::getPosition() const { return m_position; }
const Quaterniond &Body::getOrientation() const { return m_orientation; }
const Vector3d &Body::getVelocity() const { return m_velocity; }
const Vector3d &Body::getAngularVelocity() const { return m_angularVelocity; }

void Body::setPosition(const Vector3d &position) { m_position = position; }
void Body::setOrientation(const Quaterniond &orientation) { m_orientation = orientation; }
void Body::setVelocity(const Vector3d &velocity) { m_velocity = velocity; }
void Body::setAngularVelocity(const Vector3d &angularVelocity) { m_angularVelocity = angularVelocity; }

// Force accumulation
void Body::addForce(const Vector3d &force) { m_force.noalias() += force; }
void Body::clearForces() { m_force.setZero(); }
const Vector3d &Body::getForce() const { return m_force; }

void Body::addTorque(const Vector3d &torque) { m_torque.noalias() += torque; }
void Body::clearTorques() { m_torque.setZero(); }
const Vector3d &Body::getTorque() const { return m_torque; }

// Visualization convertion
glm::vec3 Body::getPositionVec3() const {
  return {m_position.x(), m_position.y(), m_position.z()};
}

glm::quat Body::getOrientationQuat() const {
  return {
    static_cast<float>(m_orientation.w()), // Cast w to float
    static_cast<float>(m_orientation.x()), // Cast x to float
    static_cast<float>(m_orientation.y()), // Cast y to float
    static_cast<float>(m_orientation.z())  // Cast z to float
};
}

} // namespace Neutron