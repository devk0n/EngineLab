#ifndef GRAVITY_H
#define GRAVITY_H

#include <unordered_set>
#include "Body.h"
#include "ForceGenerator.h"
#include "core/types.h"

namespace Neutron {

class Gravity final : public ForceGenerator {
public:
  explicit Gravity(const Vector3d& gravity) : m_gravity(gravity) {}

  void apply(double dt) override {
    for (Body* body : m_bodies) {
      // body->addForce(body->getMass() * m_gravity);
      body->addTorque(Vector3d(0, 0, 0));
    }
  }

  void addBody(Body* body) { m_bodies.insert(body); }
  void removeBody(Body* body) { m_bodies.erase(body); }
  const Vector3d& getGravity() const { return m_gravity; }
  const std::unordered_set<Body*>& getBodies() const { return m_bodies; }

private:
  Vector3d m_gravity;
  std::unordered_set<Body*> m_bodies;
};

} // namespace Neutron

#endif // GRAVITY_H
