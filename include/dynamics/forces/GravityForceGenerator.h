#ifndef GRAVITY_FORCE_GENERATOR_H
#define GRAVITY_FORCE_GENERATOR_H

#include "ForceGenerator.h"

namespace Neutron {
class GravityForceGenerator final : public ForceGenerator {
public:
  explicit GravityForceGenerator(const Vector3d& gravity) : m_gravity(gravity) {}

  void apply(double dt) override {
    for (auto& [id, body] : m_bodies) {
      body->addForce(body->getMass() * m_gravity);
    }
  }

  void addBody(Body* body) {
    m_bodies.emplace(body->getID(), body);
  }

private:
  Vector3d m_gravity;
  std::unordered_map<UniqueID, Body*> m_bodies;
};
} // namespace Neutron

#endif // GRAVITY_FORCE_GENERATOR_H
