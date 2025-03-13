// dynamics/DynamicSystem.h
#ifndef DYNAMIC_SYSTEM_H
#define DYNAMIC_SYSTEM_H
#include <chrono>
#include <memory>
#include <unordered_map>
#include <vector>
#include "Body.h"
#include "Constraint.h"
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
    const Quaterniond &orientation
  );
  Body* getBody(UniqueID ID);

  // Force and Torque management
  void addForceGenerator(const std::shared_ptr<ForceGenerator> &generator);

  // Simulation
  void step(double dt);

  void solvePositionConstraints(double epsilon, int maxIterations, double alpha,
                                double lambda, double maxCorrection);

  void solveVelocityConstraints(double epsilon, int maxIterations, double alpha,
                                double lambda, double maxCorrection);

  void clearConstraints();
  void addConstraint(const std::shared_ptr<Constraint> &constraint);

  // Utilities
  void buildMassInertiaTensor();
  const auto& getBodies() const { return m_bodies; }
  // In DynamicSystem.h

  const std::vector<std::shared_ptr<ForceGenerator>>& getForceGenerators() const { return m_forceGenerators; }
  const std::vector<std::shared_ptr<Constraint>>& getConstraints() const { return m_constraints; }

  const MatrixXd& getLastJacobian() const { return m_lastJacobian; }
  const VectorXd& getLastGamma() const { return m_lastGamma; }
  const VectorXd& getMassInertiaTensor() const { return m_massInertiaTensor; }
  const std::vector<UniqueID>& getBodyOrder() const { return m_bodyOrder; }
private:
  // Build system matrices
  void buildWrench(VectorXd &wrench); // Build every step

  std::unordered_map<UniqueID, std::unique_ptr<Body>> m_bodies;
  std::vector<std::shared_ptr<Constraint>> m_constraints;
  std::vector<std::shared_ptr<ForceGenerator>> m_forceGenerators;
  Solver m_solver;
  std::vector<UniqueID> m_bodyOrder;
  VectorXd m_massInertiaTensor;

  UniqueID m_nextID = 0;

  MatrixXd m_lastJacobian;
  VectorXd m_lastGamma;
  VectorXd m_lastForces;
};

} // namespace Neutron

#endif // DYNAMIC_SYSTEM_H