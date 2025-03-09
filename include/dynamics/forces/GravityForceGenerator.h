#ifndef GRAVITY_FORCE_GENERATOR_H
#define GRAVITY_FORCE_GENERATOR_H

#include "ForceGenerator.h"

namespace Neutron {
class GravityForceGenerator : public ForceGenerator {
public:
  explicit GravityForceGenerator(const Vector3d& gravity) : gravity(gravity) {}

  void apply(double dt) override {
    for (auto& [id, particle] : particles) {
      if (!particle->isFixed()) {
        particle->addForce(particle->getMass() * gravity);
      }
    }
  }

  void addParticle(Particle* particle) {
    particles.emplace(particle->getID(), particle);
  }

private:
  Vector3d gravity;
  std::unordered_map<UniqueID, Particle*> particles;
};
} // namespace Neutron

#endif // GRAVITY_FORCE_GENERATOR_H
