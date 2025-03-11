#include "Particle.h"

namespace Neutron {

Particle::Particle(
  const UniqueID ID,
  const double mass,
  const Vector3d &position)
  : m_ID(ID),
    m_mass(mass),
    m_position(position),
    m_velocity(Vector3d::Zero()),
    m_force(Vector3d::Zero()) {}

UniqueID Particle::getID() const { return m_ID; }

// Physical properties
double Particle::getMass() const { return m_mass; }

// State variables
const Vector3d &Particle::getPosition() const { return m_position; }
const Vector3d &Particle::getVelocity() const { return m_velocity; }

void Particle::setPosition(const Vector3d &position) { m_position = position; }
void Particle::setVelocity(const Vector3d &velocity) { m_velocity = velocity; }

// Force accumulation
void Particle::addForce(const Vector3d &force) { m_force.noalias() += force; }
void Particle::clearForces() { m_force.setZero(); }
const Vector3d &Particle::getForce() const { return m_force; }

// Configuration
void Particle::setFixed(const bool fixed) { m_fixed = fixed; }
bool Particle::isFixed() const { return m_fixed; }

// Visualization convertion
glm::vec3 Particle::getPositionVec3() const {
  return glm::vec3(m_position.x(), m_position.y(), m_position.z());
}

} // namespace Neutron
