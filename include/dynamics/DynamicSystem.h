// dynamics/DynamicSystem.h
#ifndef DYNAMIC_SYSTEM_H
#define DYNAMIC_SYSTEM_H

#include "Body.h"
#include "ForceGenerator.h"
#include "Solver.h"
#include "core/types.h"

namespace Neutron {

class DynamicSystem {
public:
  DynamicSystem();
  ~DynamicSystem() = default;

  // Body management
  UniqueID addBody(
    const double &mass,
    const Vector3d &inertia,
    const Vector3d &position,
    const Vector4d &orientation
  );
  Body* getBody(UniqueID ID);

  // Simulation
  void step(double dt);

  void solvePositionConstraints(double epsilon, int maxIterations, double alpha,
                                double lambda, double maxCorrection);

  const Body* getBody(UniqueID ID) const;

  void addConstraint(const std::shared_ptr<Constraint> &constraint);

  const std::vector<std::shared_ptr<Constraint>> &getConstraints() const;

  void addForceGenerator(const std::shared_ptr<ForceGenerator>& generator) {
    m_forceGenerators.emplace_back(generator);
  }

  void projectPositions(VectorXd& q, int maxIterations);
  void projectVelocities(VectorXd& q, VectorXd& q_dot);

  VectorXd computeAccelerations(const VectorXd& q, const VectorXd& q_dot);
  void setVelocityState(const VectorXd& velocities) const;
  void getVelocityState(VectorXd& velocities) const;

  const std::vector<std::unique_ptr<Body>>& getBodies() const { return m_bodies; }

private:
  void getSystemState(VectorXd &state) const;
  void setSystemState(VectorXd state);
  // Build system matrices
  void buildMassInertiaTensor();
  void buildWrench(VectorXd& wrench) const;
  void buildConstraints(
    VectorXd &phi,
    MatrixXd &jacobian, VectorXd &gamma, VectorXd &accelerations, VectorXd &
    lambdas);

  // System state
  std::vector<std::unique_ptr<Body>> m_bodies;
  std::unordered_map<UniqueID, size_t> m_bodyIndex;
  std::vector<std::shared_ptr<ForceGenerator>> m_forceGenerators;
  std::vector<std::shared_ptr<Constraint>> m_constraints;
  int m_numConstraints = 0;
  int m_numBodies = 0;
  VectorXd m_massInertiaTensor;

  // Solver and system matrices
  Solver m_solver;
  bool m_dirty = true;

  UniqueID m_nextID = 0;
};

} // namespace Neutron

#endif // DYNAMIC_SYSTEM_H