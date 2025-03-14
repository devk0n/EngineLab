#ifndef DISTANCE_CONSTRAINT_H
#define DISTANCE_CONSTRAINT_H
#include <Body.h>
#include <Constraint.h>

namespace Neutron {
class DistanceConstraint final : public Constraint {
public:
  DistanceConstraint(
    Body *body1,
    Body *body2,
    double distance
  );

  void computePhi(VectorXd &phi, int startRow) override;
  void computeJacobian(MatrixXd &jacobian, int startRow) override;
  void computeGamma(VectorXd &gamma, int startRow) override;

private:
  Body* m_body1;
  Body* m_body2;
  double m_distance;
};
}

#endif // DISTANCE_CONSTRAINT_H
