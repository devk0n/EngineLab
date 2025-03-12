// dynamics/DynamicSystem.cpp
#include "DynamicSystem.h"
#include <algorithm>
#include "utils/Logger.h"

namespace Neutron {
DynamicSystem::DynamicSystem() = default;

UniqueID DynamicSystem::addParticle(
  double mass,
  const Vector3d &position) {
  UniqueID id = m_nextID++;
  auto particle = std::make_unique<Particle>(id, mass, position);
  m_particles[id] = std::move(particle);
  return id;
}

Particle *DynamicSystem::getParticle(const UniqueID id) {
  if (auto it = m_particles.find(id); it != m_particles.end()) {
    return it->second.get();
  }
  return nullptr;
}

UniqueID DynamicSystem::addBody(
  double mass,
  const Vector3d &inertia,
  const Vector3d &position,
  const Quaterniond &orientation
) {
  UniqueID id = m_nextID++;
  auto body = std::make_unique<Body>(id, mass, inertia, position, orientation);
  m_bodies[id] = std::move(body);
  return id;
}

Body *DynamicSystem::getBody(const UniqueID id) {
  if (auto it = m_bodies.find(id); it != m_bodies.end()) {
    return it->second.get();
  }
  return nullptr;
}

void DynamicSystem::addConstraint(
  const std::shared_ptr<Constraint> &constraint) {
  m_constraints.push_back(constraint);
  m_constraintSolver.addConstraint(constraint);
}

void DynamicSystem::clearConstraints() {
  m_constraints.clear();
  m_constraintSolver.clearConstraints();
}

void DynamicSystem::addForceGenerator(
  const std::shared_ptr<ForceGenerator> &generator) {
  m_forceGenerators.push_back(generator);
}

void DynamicSystem::buildMassInertiaTensor() {
  int n = static_cast<int>(m_bodies.size()) * 6; // 6 DOF per body
  m_massInertiaTensor.resize(n);
  m_massInertiaTensor.setZero();

  int i = 0;
  for (const auto &[id, body]: m_bodies) {
    m_massInertiaTensor.segment<3>(i * 6).setConstant(body->getMass());
    m_massInertiaTensor.segment<3>(i * 6 + 3) = body->getInertia();
    ++i;
  }
}

void DynamicSystem::buildWrench(VectorXd &wrench) {
  int n = static_cast<int>(m_bodies.size()) * 6; // 6 DOF per body
  wrench.resize(n);
  wrench.setZero();

  int i = 0;
  for (const auto &[id, body]: m_bodies) {
    wrench.segment<3>(i * 6) = body->getForce();
    wrench.segment<3>(i * 6 + 3) = body->getTorque() - body->getGyroscopicTorque();
    ++i;
  }
}

void DynamicSystem::step(const double dt) {
  if (m_bodies.empty()) return;

  // Clear and apply forces
  for (auto &[id, body]: m_bodies) { body->clearForces(); }
  for (auto &generator: m_forceGenerators) { generator->apply(dt); }

  // Build system matrices
  if (m_massInertiaTensor.size() == 0) { buildMassInertiaTensor(); }

  // Solve dynamics
  std::vector<Body *> bodies;
  for (const auto &[id, body]: m_bodies) {
    bodies.push_back(body.get());
  }

  VectorXd forces;
  buildWrench(forces);

  MatrixXd jacobian;
  VectorXd constraintRHS;
  m_constraintSolver.buildJacobian(
      bodies,
      jacobian,
      constraintRHS
  );

  VectorXd accelerations;
  VectorXd lambdas;
  m_constraintSolver.solveSystem(
      m_massInertiaTensor,
      forces,
      jacobian,
      constraintRHS,
      accelerations,
      lambdas
  );

  // Update positions and velocities
  for (int i = 0; i < bodies.size(); ++i) {
    bodies[i]->setVelocity(bodies[i]->getVelocity() + accelerations.segment<3>(i * 6) * dt);
    bodies[i]->setAngularVelocity(bodies[i]->getAngularVelocity() + accelerations.segment<3>(i * 6 + 3) * dt);
    bodies[i]->setPosition(bodies[i]->getPosition() + bodies[i]->getVelocity() * dt);

    // Update orientation using quaternion integration
    Vector3d angularVelocity = bodies[i]->getAngularVelocity();
    double angularSpeed = angularVelocity.norm();

    if (angularSpeed > 1e-10) { // Avoid division by zero
      Vector3d axis = angularVelocity / angularSpeed; // Normalized rotation axis
      double angle = angularSpeed * dt; // Angle of rotation

      // Create a quaternion representing the rotation
      Quaterniond deltaQ;
      deltaQ = Eigen::AngleAxisd(angle, axis);

      // Update the orientation
      Quaterniond currentOrientation = bodies[i]->getOrientation();
      Quaterniond newOrientation = currentOrientation * deltaQ;
      newOrientation.normalize(); // Normalize to ensure it remains a unit quaternion
      bodies[i]->setOrientation(newOrientation);
    }
  }

  // --- Constraint Stabilization ---
  solvePositionConstraints(
    1e-6,    // epsilon
    100,       // maxIterations
    0.1,     // alpha
    1e-6,    // lambda
    0.05     // maxCorrection
  );

  solveVelocityConstraints(
    1e-6,    // epsilon
    100,       // maxIterations
    0.1,     // alpha
    1e-6,    // lambda
    0.05     // maxCorrection
  );
}
void DynamicSystem::solvePositionConstraints(
    const double epsilon = 1e-6,
    const int maxIterations = 10,
    const double alpha = 0.5,         // Relaxation parameter
    const double lambda = 1e-6,       // Use damped least squares
    const double maxCorrection = 0.1) // Maximum velocity correction per iteration
{
  if (m_bodies.empty()) return;

  std::vector<Body *> bodies;
  for (const auto &[id, body]: m_bodies) {
    bodies.push_back(body.get());
  }

  // Smaller value = more stable but slower convergence
  for (int iter = 0; iter < maxIterations; ++iter) {
    // Build Jacobian and constraint equations
    MatrixXd jacobian;
    VectorXd constraintRHS;
    m_constraintSolver.buildJacobian(bodies, jacobian, constraintRHS);

    // Calculate constraint violations
    VectorXd c(constraintRHS.size());
    int itt = 0;
    for (const auto &constraint: m_constraints) {
      constraint->computeConstraintEquations(c, itt);
      ++itt;
    }

    // Check if constraints are satisfied
    double error = c.norm();
    if (error < epsilon) {
      break;
    }

    // Use damped least squares (Levenberg-Marquardt)
    MatrixXd JTJ = jacobian.transpose() * jacobian;
    MatrixXd H = JTJ + lambda * MatrixXd::Identity(JTJ.rows(), JTJ.cols());
    VectorXd dx = H.ldlt().solve(jacobian.transpose() * (-c));

    // Apply scaled corrections to positions
    int i = 0;
    for (auto &body: bodies) {
      Vector3d correction = alpha * dx.segment<3>(i * 6);

      // Limit correction magnitude
      if (correction.norm() > maxCorrection) {
        correction *= maxCorrection / correction.norm();
      }
      body->setPosition(body->getPosition() + correction);
      i++;
    }
  }
}

void DynamicSystem::solveVelocityConstraints(
    const double epsilon = 1e-6,
    const int maxIterations = 10,
    const double alpha = 0.5,         // Relaxation parameter
    const double lambda = 1e-6,       // Use damped least squares
    const double maxCorrection = 0.1) // Maximum velocity correction per iteration
{
  if (m_bodies.empty()) return;

  std::vector<Body *> bodies;
  for (const auto &[id, body]: m_bodies) {
    bodies.push_back(body.get());
  }

  for (int iter = 0; iter < maxIterations; ++iter) {
    // Build Jacobian
    MatrixXd jacobian;
    VectorXd dummy;
    m_constraintSolver.buildJacobian(bodies, jacobian, dummy);

    // Compute current velocity constraint violation
    VectorXd qdot(bodies.size() * 6);
    for (int i = 0; i < bodies.size(); ++i) {
      qdot.segment<3>(i * 6) = bodies[i]->getVelocity();
      qdot.segment<3>(i * 6 + 3) = bodies[i]->getAngularVelocity();
    }

    VectorXd cdot = jacobian * qdot;

    // Check if velocity constraints are satisfied
    if (cdot.norm() < epsilon) {
      break;
    }

    MatrixXd JTJ = jacobian.transpose() * jacobian;
    MatrixXd H = JTJ + lambda * MatrixXd::Identity(JTJ.rows(), JTJ.cols());
    VectorXd dv = H.ldlt().solve(jacobian.transpose() * (-cdot));

    // Apply scaled corrections to velocities
    int i = 0;
    for (auto &body: bodies) {
      Vector3d correction = alpha * dv.segment<3>(i * 6);

      // Limit correction magnitude
      if (correction.norm() > maxCorrection) {
        correction *= maxCorrection / correction.norm();
      }
      body->setVelocity(body->getVelocity() + correction);
      body->setAngularVelocity(body->getAngularVelocity() + dv.segment<3>(i * 6 + 3));
      i++;
    }
  }
}

} // namespace Neutron
