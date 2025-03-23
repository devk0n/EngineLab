#ifndef BODY_H
#define BODY_H

#include <utility>

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
      Vector3d position)
      : m_ID(ID),
        m_index(index),
        m_mass(mass),
        m_inverseMass(1/mass),
        m_position(std::move(position)) {}

  // Getters for state
  UniqueID getID() const { return m_ID; }
  int getIndex() const { return m_index; }
  Vector3d getPosition() const { return m_position; }
  Vector3d getVelocity() const { return m_velocity; }
  double getMass() const { return m_mass; }
  double getInverseMass() const { return m_inverseMass; }

  // Setters for state
  void setPosition(const Vector3d &position) { m_position = position; }
  void setVelocity(const Vector3d &velocity) { m_velocity = velocity; }

  // Force accumulation
  void addForce(const Vector3d& force) { m_force.noalias() += force; }
  void clearForces() { m_force.setZero(); }
  const Vector3d& getForce() const { return m_force; }

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
      static_cast<float>(0), // Cast x to float
      static_cast<float>(0), // Cast y to float
      static_cast<float>(0),  // Cast z to float
      static_cast<float>(1)
  };
  }

private:
  UniqueID m_ID = -1;
  int m_index = -1;

  // Physical properties
  double m_mass;
  double m_inverseMass;

  // State variables
  Vector3d m_position{Vector3d::Zero()};
  Vector3d m_velocity{Vector3d::Zero()};

  // Force accumulators
  Vector3d m_force{Vector3d::Zero()};

  bool m_fixed = false;

};
} // Proton

#endif // BODY_H
