// dynamics/Constraint.h
#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <map>
#include "Particle.h"
#include "types.h"

namespace Neutron {

class Constraint {
public:
  explicit Constraint(const int dofs) : m_dofs(dofs) {}
  virtual ~Constraint() = default;

  // Compute constraint equation value (C)
  // C=0 means the constraint is satisfied
  virtual void computeConstraintEquations(VectorXd& c, int startRow) = 0;

  // Compute Jacobian of constraint (∂C/∂q)
  virtual void computeJacobian(MatrixXd& jacobian, int startRow, const std::map<Particle*, int>& particleToIndex) = 0;

  // Compute the time derivative of Jacobian times qdot (J_dot * qdot)
  // This is needed for the right side of the constraint equation
  virtual void computeJacobianDerivative(VectorXd& jdotqdot, int startRow) = 0;

  // Number of constraint equations
  [[nodiscard]] int getDOFs() const { return m_dofs; }

protected:
  int m_dofs; // Degrees of freedom removed by this constraint
};

} // namespace Neutron

#endif // CONSTRAINT_H