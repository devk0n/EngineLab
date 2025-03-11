#ifndef PARTICLE_H
#define PARTICLE_H

#include <glm/vec3.hpp>
#include "core/types.h"

namespace Neutron {

class Particle {
public:
  Particle(
      UniqueID ID,
      double mass,
      const Vector3d &position
  );

  UniqueID getID() const;

  // Physical properties
  double getMass() const;

  // State variables
  const Vector3d &getPosition() const;
  const Vector3d &getVelocity() const;

  void setPosition(const Vector3d &position);
  void setVelocity(const Vector3d &velocity);

  // Force accumulation
  void addForce(const Vector3d &force);
  void clearForces();
  const Vector3d &getForce() const;

  // Configuration
  void setFixed(bool fixed);
  bool isFixed() const;

  // Visualization convertion
  glm::vec3 getPositionVec3() const;

private:
  UniqueID m_ID;

  // Physical properties
  double m_mass;

  // State variables
  Vector3d m_position;
  Vector3d m_velocity;

  // Force accumulators
  Vector3d m_force;

  bool m_fixed = false;
};

} // namespace Neutron

#endif // PARTICLE_H
