#include "DistanceConstraint.h"

namespace Neutron {

DistanceConstraint::DistanceConstraint(
  Particle *particle1,
  Particle *particle2,
  const double distance)
  : Constraint(1),
    m_particle1(particle1),
    m_particle2(particle2),
    m_distance(distance) {}

DistanceConstraint::DistanceConstraint(
  Particle *particle1,
  Particle *particle2)
  : Constraint(1),
    m_particle1(particle1),
    m_particle2(particle2) {
  // Compute distance
  Vector3d d = m_particle2->getPosition() - m_particle1->getPosition();
  m_distance = d.norm();
}

void DistanceConstraint::computeConstraintEquations(VectorXd &c, int startRow) {
  // Relative position
  Vector3d d = m_particle2->getPosition() - m_particle1->getPosition();

  // Constraint equation
  c[0] = d.transpose() * d - (m_distance * m_distance);
}

void DistanceConstraint::computeJacobian(
    MatrixXd& jacobian,
    const int startRow,
    const std::map<Particle*, int>& particleToIndex) {
  Vector3d d = m_particle2->getPosition() - m_particle1->getPosition();
  const int index1 = particleToIndex.at(m_particle1);
  const int index2 = particleToIndex.at(m_particle2);
  jacobian.block<1, 3>(startRow, index1 * 3) = -2 * d.transpose(); // Correct sign for particle 1
  jacobian.block<1, 3>(startRow, index2 * 3) =  2 * d.transpose(); // Correct sign for particle 2
}

void DistanceConstraint::computeJacobianDerivative(
    VectorXd& jdotqdot,
    int startRow) {
  Vector3d v = m_particle2->getVelocity() - m_particle1->getVelocity();
  const double result = 2.0 * v.squaredNorm(); // Correct sign to positive
  jdotqdot[startRow] = result;
}


} // namespace Neutron
