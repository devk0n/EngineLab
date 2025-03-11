#ifndef SPRING_FORCE_GENERATOR_H
#define SPRING_FORCE_GENERATOR_H

#include "ForceGenerator.h"

namespace Neutron {
class SpringForceGenerator final : public ForceGenerator {
public:
  SpringForceGenerator(
      Particle* p1,
      Particle* p2,
      double restLength,
      double stiffness,
      double damping = 0.1)
      : m_p1(p1),
        m_p2(p2),
        m_restLength(restLength),
        m_stiffness(stiffness),
        m_damping(damping) {

  }

  void apply(double dt) override {
    Vector3d delta = m_p2->getPosition() - m_p1->getPosition();
    double currentLength = delta.norm();
    if (currentLength < 1e-6) {
      return; // Prevent division by zero and no compression force
    }

    Vector3d direction = delta / currentLength;

    // Hooke's Law (spring force)
    double springForce = m_stiffness * (currentLength - m_restLength);

    // Damping force (proportional to relative velocity)
    Vector3d relativeVel = m_p2->getVelocity() - m_p1->getVelocity();
    double dampingForce = m_damping * relativeVel.dot(direction);

    // Total force
    double totalForce = springForce + dampingForce;
    Vector3d force = direction * totalForce;

    m_p1->addForce(force);
    m_p2->addForce(-force);
  }

private:
  Particle* m_p1;
  Particle* m_p2;
  double m_restLength;
  double m_stiffness;
  double m_damping;
};
} // namespace Neutron

#endif // SPRING_FORCE_GENERATOR_H
