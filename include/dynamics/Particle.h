#ifndef PARTICLE_H
#define PARTICLE_H
#include <glm/vec3.hpp>
#include "types.h"

namespace Neutron {

class Particle {
public:
  Particle(
      UniqueID id,
      double mass,
      const Vector3d &position
  );

  // Getters for state
  [[nodiscard]] UniqueID getID() const { return m_id; }
  [[nodiscard]] const Vector3d &getPosition() const { return m_position; }
  [[nodiscard]] glm::vec3 getPositionVec3() const {
    return {m_position[0], m_position[1], m_position[2]};
  }
  [[nodiscard]] const Vector3d &getVelocity() const { return m_velocity; }

  // Mass properties
  [[nodiscard]] double getMass() const { return m_mass; }

  // Sets the current position/velocity vector
  void setPosition(const Vector3d &position) { m_position = position; }
  void setVelocity(const Vector3d &velocity) { m_velocity = velocity; }

  // Force accumulation
  void addForce(const Vector3d &force) { m_force.noalias() += force; }
  void clearForces() { m_force.setZero(); }

  // Get the accumulated force
  [[nodiscard]] const Vector3d &getForce() const { return m_force; }

  // Fixed
  void setFixed(const bool fixed) { m_fixed = fixed; }
  [[nodiscard]] bool isFixed() const { return m_fixed; }

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
