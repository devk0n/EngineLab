#include "Particle.h"

namespace Neutron {

Particle::Particle(
  const UniqueID id,
  const double mass,
  const Vector3d &position)
  : m_id(id),
    m_mass(mass),
    m_position(position),
    m_velocity(Vector3d::Zero()),
    m_force(Vector3d::Zero()) {}

} // namespace Neutron
