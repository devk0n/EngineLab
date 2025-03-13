// dynamics/Constraint.h
#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include <map>
#include <Particle.h>

#include "Body.h"
#include "core/types.h"

namespace Neutron {

class Constraint {
public:
  explicit Constraint(const int DOFs) : m_DOFs(DOFs) {}
  virtual ~Constraint() = default;

  // Compute constraint equation value (C)
  // C=0 means the constraint is satisfied
  virtual void computePhi(VectorXd& phi, int startRow) = 0;

  // Compute Jacobian of constraint (∂C/∂q)
  virtual void computeJacobian(MatrixXd& jacobian, int startRow) = 0;

  // Compute the time derivative of Jacobian times qdot (J_dot * qdot)
  // This is needed for the right side of the constraint equation
  virtual void computeGamma(VectorXd& gamma, int startRow) = 0;

  // Number of constraint equations
  [[nodiscard]] int getDOFs() const { return m_DOFs; }

protected:
  int m_DOFs; // Degrees of freedom removed by this constraint
};

} // namespace Neutron

#endif // CONSTRAINT_H