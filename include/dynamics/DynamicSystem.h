// dynamics/DynamicSystem.h
#ifndef DYNAMIC_SYSTEM_H
#define DYNAMIC_SYSTEM_H

#include "Body.h"
#include "Solver.h"
#include "core/types.h"

namespace Neutron {

class DynamicSystem {
public:
  DynamicSystem();

  void buildMassTensor();

  ~DynamicSystem() = default;

  // Body management
  UniqueID addBody(
    const double &mass,
    const Vector3d &inertia,
    const Vector3d &position,
    const Vector4d &orientation
  );
  Body* getBody(UniqueID ID);
  const Body* getBody(UniqueID ID) const;
  const std::vector<std::unique_ptr<Body>>& getBodies() const { return m_bodies; }

  // Simulation
  void step(double dt);

  void computeAccelerations(const VectorXd& q, const VectorXd& q_dot);

  const VectorXd& getMassInertiaTensor() const { return m_massInertiaTensor; }
  const MatrixXd& getJacobian() const { return m_jacobian; }
  const VectorXd& getPhi() const { return m_phi; }
  const VectorXd& getGamma() const { return m_gamma; }
  const VectorXd& getVelocityState() const { return m_velocityState; }
  const VectorXd& getSystemState() const { return m_systemState; }

  void addConstraint(const std::shared_ptr<Constraint> &constraint);

private:
  // Build system matrices
  void buildMassInertiaTensor();

  void getVelocityState(VectorXd& velocities) const;
  void setVelocityState(const VectorXd& velocities) const;
  void getSystemState(VectorXd &state) const;
  void setSystemState(VectorXd state);


  // System state
  std::vector<std::unique_ptr<Body>> m_bodies;
  std::unordered_map<UniqueID, size_t> m_bodyIndex;
  std::vector<std::shared_ptr<Constraint>> m_constraints;

  VectorXd m_massInertiaTensor = VectorXd::Zero(0);
  MatrixXd m_jacobian = MatrixXd::Zero(0, 0);
  VectorXd m_phi = VectorXd::Zero(0);
  VectorXd m_gamma = VectorXd::Zero(0);
  VectorXd m_velocityState = VectorXd::Zero(0);
  VectorXd m_systemState = VectorXd::Zero(0);

  int m_numConstraints = 0;
  // Solver and system matrices
  Solver m_solver;

  UniqueID m_nextID = 0;
  int m_numBodies = 0;
  bool m_dirty = true;
};

} // namespace Neutron

#endif // DYNAMIC_SYSTEM_H