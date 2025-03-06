#ifndef SYSTEMSOLVER_H
#define SYSTEMSOLVER_H

#include "SystemConfiguration.h"

class SystemSolver {
public:
  explicit SystemSolver(SystemConfiguration& system, glm::vec3 gravity = {0.0, 0.0, 0.0})
      : m_system(system), m_gravity(gravity) {}

  void update(float dt) {
    for (auto& [name, body] : m_system.bodies()) {
      if (body.mass <= 0.0f) continue;  // Ignore static objects

      // Apply gravity
      body.acceleration = m_gravity;

      // Integrate velocity using Euler method
      body.velocity += body.acceleration * dt;

      // Integrate position
      body.position += body.velocity * dt;
    }
  }

private:
  SystemConfiguration& m_system;
  glm::vec3 m_gravity;
};

#endif // SYSTEMSOLVER_H
