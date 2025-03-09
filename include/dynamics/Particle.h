#ifndef PARTICLE_H
#define PARTICLE_H
#include <core/types.h>

#include "glm/vec3.hpp"

namespace Neutron {

class Particle {
public:
  Particle(
      UniqueID id,
      double mass,
      const Vector3d &position
  );

  // Getters for state
  UniqueID getID() const { return m_id; }
  const Vector3d &getPosition() const { return m_position; }
  glm::vec3 getPositionVec3() const {
    return glm::vec3(m_position[0], m_position[1], m_position[2]);
  }
  const Vector3d &getVelocity() const { return m_velocity; }

  // Mass properties
  double getMass() const { return m_mass; }

  // Sets the current position/velocity vector
  void setPosition(const Vector3d &position) { m_position = position; }
  void setVelocity(const Vector3d &velocity) { m_velocity = velocity; }

  // Force accumulation
  void addForce(const Vector3d &force) { m_force += force; }
  void clearForces() { m_force.setZero(); }

  // Get the accumulated force
  const Vector3d &getForce() const { return m_force; }

  // Fixed
  void setFixed(bool fixed) { m_fixed = fixed; }
  bool isFixed() const { return m_fixed; }

private:
  UniqueID m_id;

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
