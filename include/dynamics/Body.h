#ifndef BODY_H
#define BODY_H
#include <core/types.h>

namespace Neutron {

class Body {
public:
  Body(
      UniqueID ID,
      double mass,
      Matrix3d inertia,
      Vector3d position,
      Quaterniond orientation
  );

private:
  UniqueID m_ID;

  // Physical properties
  double m_mass;
  Matrix3d m_inertia;

  // State variables
  Vector3d m_position;
  Vector3d m_velocity;
  Quaterniond m_orientation;
  Vector3d m_angularVelocity;

  // Force accumulators
  Vector3d m_force;
  Vector3d m_torque;
};

} // namespace Neutron

#endif // BODY_H
