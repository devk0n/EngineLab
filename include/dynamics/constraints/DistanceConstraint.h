#ifndef DISTANCE_CONSTRAINT_H
#define DISTANCE_CONSTRAINT_H

#include <map>

#include "dynamics/Constraint.h"
#include "dynamics/Particle.h"

namespace Neutron {
class DistanceConstraint final : public Constraint {
public:
  DistanceConstraint(
      Particle* particle1,
      Particle* particle2,
      double distance
  );

  DistanceConstraint(
      Particle* particle1,
      Particle* particle2
  );

  void computeConstraintEquations(VectorXd& c, int startRow) override;

  void computeJacobian(MatrixXd& jacobian, int startRow, const std::map<Particle*, int>& particleToIndex) override;

  void computeJacobianDerivative(VectorXd& jdotqdot, int startRow) override;

  [[nodiscard]] Particle* getParticle1() const { return m_particle1; }
  [[nodiscard]] Particle* getParticle2() const { return m_particle2; }
  [[nodiscard]] double getDesiredDistance() const { return m_distance; }

private:
  Particle* m_particle1;
  Particle* m_particle2;
  double m_distance;
};
} // namespace Neutron

#endif // DISTANCE_CONSTRAINT_H
