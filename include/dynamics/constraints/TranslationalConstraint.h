#ifndef TRANSLATIONAL_CONSTRAINT_H
#define TRANSLATIONAL_CONSTRAINT_H

#include "dynamics/Constraint.h"
#include "dynamics/Particle.h"

namespace Neutron {
class TranslationalConstraint final : public Constraint {
public:
  TranslationalConstraint(Particle* particle, const Vector3d& axis, double targetPosition = 0.0);
  void computeConstraintEquations(VectorXd& c, int startRow) override;
  void computeJacobian(MatrixXd& jacobian, int startRow, const std::map<Particle*, int>& particleToIndex) override;
  void computeJacobianDerivative(VectorXd& jdotqdot, int startRow) override;

private:
  Particle* m_particle;  // Particle to constrain
  Vector3d m_axis;       // Axis or plane normal (e.g., (1, 0, 0) for X-axis)
  double m_target;       // Target position along the axis
};
} // namespace Neutron

#endif // TRANSLATIONAL_CONSTRAINT_H
