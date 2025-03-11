#include "TranslationalConstraint.h"

namespace Neutron {
TranslationalConstraint::TranslationalConstraint(Particle* particle, const Vector3d& axis, double targetPosition)
: Constraint(1), m_particle(particle), m_axis(axis.normalized()), m_target(targetPosition) {}

void TranslationalConstraint::computeConstraintEquations(VectorXd& c, int startRow) {
  // Constraint equation: (p - p₀) · n = 0
  c[startRow] = (m_particle->getPosition().dot(m_axis) - m_target);
}

void TranslationalConstraint::computeJacobian(MatrixXd& jacobian, int startRow, const std::map<Particle*, int>& particleToIndex) {
  int index = particleToIndex.at(m_particle);
  jacobian.block<1, 3>(startRow, index * 3) = m_axis.transpose();
}

void TranslationalConstraint::computeJacobianDerivative(VectorXd& jdotqdot, int startRow) {
  // No velocity-dependent terms for this constraint
  jdotqdot[startRow] = 0.0;
}
}