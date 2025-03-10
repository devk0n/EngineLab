// dynamics/DynamicSystem.h
#ifndef DYNAMIC_SYSTEM_H
#define DYNAMIC_SYSTEM_H
#include <chrono>
#include <memory>
#include <unordered_map>
#include <vector>
#include "Constraint.h"
#include "ConstraintSolver.h"
#include "ForceGenerator.h"
#include "Particle.h"
#include "types.h"

namespace Neutron {

class DynamicSystem {
public:
  DynamicSystem();
  ~DynamicSystem() = default;

  // Particle management
  UniqueID addParticle(
      double mass,
      const Vector3d& position
  );
  Particle* getParticle(UniqueID id);

  // Constraint management
  void addConstraint(const std::shared_ptr<Constraint>& constraint);
  void clearConstraints();

  void addForceGenerator(const std::shared_ptr<ForceGenerator>& generator);

  // External forces
  // void addForceToBody(UniqueID bodyID, const Vector3d& force, const Vector3d& worldPoint);

  // Simulation
  void step(double dt);

  const auto& getParticles() const { return m_particles; }
  const auto& getConstraints() const { return m_constraints; }

private:
  void calculateDerivatives(const VectorXd& state, VectorXd& derivatives);
  void setSystemState(const VectorXd& state);
  void integrateRK4(double dt);
  void getSystemState(VectorXd& state);

  // Build system matrices
  void buildMassMatrix(); // Build once
  void buildForceVector(VectorXd& forces); // Build every step
  void buildConstraintRHS(VectorXd& constraintRHS, double alpha, double beta);

  // Integration step
  void integrate(double dt);

  std::unordered_map<UniqueID, std::unique_ptr<Particle>> m_particles;
  std::vector<std::shared_ptr<Constraint>> m_constraints;
  std::vector<std::shared_ptr<ForceGenerator>> m_forceGenerators;
  ConstraintSolver m_constraintSolver;

  VectorXd m_massMatrix;
  UniqueID m_nextID = 0;
};

} // namespace Neutron

#endif // DYNAMIC_SYSTEM_H