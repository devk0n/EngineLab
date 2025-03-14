#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "core/types.h"

namespace Neutron {
class Constraint {
public:
  Constraint(int DOFs) : m_DOFs(DOFs) {}
  virtual ~Constraint() = default;

  virtual void computePhi(VectorXd& phi, int startRow) = 0;
  virtual void computeJacobian(MatrixXd& jacobian, int startRow) = 0;
  virtual void computeGamma(VectorXd& gamma, int startRow) = 0;

  // Number of constraint equations
  int getDOFs() const { return m_DOFs; }

protected:
  int m_DOFs; // Degrees of freedom removed by this constraint
};
} // namespace Neutron

#endif // CONSTRAINT_H