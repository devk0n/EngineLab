// dynamics/DynamicSystem.h
#ifndef DYNAMIC_SYSTEM_H
#define DYNAMIC_SYSTEM_H
#include <chrono>
#include <memory>
#include <unordered_map>
#include <vector>
#include "Body.h"
#include "Constraint.h"
#include "ConstraintSolver.h"
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

  // Utilities
  void buildMassInertiaTensor();
  const auto& getBodies() const { return m_bodies; }

private:
  // Build system matrices
  void buildWrench(VectorXd &wrench); // Build every step

  std::unordered_map<UniqueID, std::unique_ptr<Body>> m_bodies;
  std::vector<std::shared_ptr<ForceGenerator>> m_forceGenerators;
  Solver m_solver;

  VectorXd m_massInertiaTensor;

  UniqueID m_nextID = 0;
};

} // namespace Neutron

#endif // DYNAMIC_SYSTEM_H