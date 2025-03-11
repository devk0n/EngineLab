#ifndef BODY_H
#define BODY_H

#include <glm/vec3.hpp>
#include "core/types.h"

namespace Neutron {

class Body {
public:
  Body(
      UniqueID ID,
      double mass,
      const Matrix3d &inertia,
      const Vector3d &position,
      const Quaterniond &orientation
  );

  UniqueID getID() const;

  // Physical properties
  double getMass() const;
  const Matrix3d &getInertia() const;

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

private:
  UniqueID m_ID;

  // Physical properties
  double m_mass;
  Matrix3d m_inertia;

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
