// dynamics/DynamicSystem.cpp
#include "DynamicSystem.h"
#include <algorithm>
#include "utils/Logger.h"

namespace Neutron {
DynamicSystem::DynamicSystem() = default;

UniqueID DynamicSystem::addBody(
  const double &mass,
  const Vector3d &inertia,
  const Vector3d &position,
  const Quaterniond &orientation
) {
  UniqueID ID = m_nextID++;
  auto body = std::make_unique<Body>(ID, mass, inertia, position, orientation);
  m_bodies[ID] = std::move(body);
  m_bodyOrder.push_back(ID);
  return ID;
}

Body *DynamicSystem::getBody(const UniqueID ID) {
  if (auto it = m_bodies.find(ID); it != m_bodies.end()) {
    return it->second.get();
  }
  return nullptr;
}

void DynamicSystem::addForceGenerator(
  const std::shared_ptr<ForceGenerator> &generator) {
  m_forceGenerators.emplace_back(generator);
}

void DynamicSystem::addConstraint(const std::shared_ptr<Constraint> &constraint) {
  m_constraints.emplace_back(constraint);
  m_solver.addConstraint(constraint);
}

void DynamicSystem::clearConstraints() {
  m_constraints.clear();
  m_solver.clearConstraints();
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
    wrench.segment<3>(i * 6 + 3) = body->getTorque();
    ++i;
  }
}

void DynamicSystem::step(const double dt) {
  if (m_bodies.empty()) return;


  // --- 1. Clear Forces and Apply External Forces ---
  for (auto &[id, body]: m_bodies) {
    body->clearForces();
    body->clearTorques();
  }

  for (auto &generator: m_forceGenerators) {
    generator->apply(dt);
  }

  if (m_massInertiaTensor.size() == 0) {
    buildMassInertiaTensor();
  }

  VectorXd forces, gamma, accelerations, lambdas;
  MatrixXd jacobian;
  buildWrench(forces);
  accelerations.resize(forces.size());
  lambdas.resize(m_constraints.size());

  m_solver.buildJacobian(jacobian, gamma, m_massInertiaTensor.size());

  m_solver.solveSystem(
    m_massInertiaTensor,
    forces,
    jacobian,
    gamma,
    accelerations,
    lambdas
  );

  // --- 2. Update State Variables ---
  int i = 0;
  for (auto &[id, body] : m_bodies) {
    if (!body->isFixed()) {
      body->setVelocity(body->getVelocity() + accelerations.segment<3>(i * 6) * dt);
      body->setAngularVelocity(body->getAngularVelocity() + accelerations.segment<3>(i * 6 + 3) * dt);
    }
    ++i; // Track the iteration index
  }

  // --- 3. Update Position ---
  for (auto &[id, body]: m_bodies) {
    if (!body->isFixed()) {
      // Update position
      body->setPosition(body->getPosition() + body->getVelocity() * dt);

      // Update orientation
      Vector3d angularVelocity = body->getAngularVelocity(); // Angular velocity in body frame
      Quaterniond currentOrientation = body->getOrientation(); // Current orientation as a quaternion

      // Compute rotation vector
      Vector3d deltaTheta = angularVelocity * dt; // Rotation vector

      // Convert rotation vector to quaternion (exponential map)
      double theta = deltaTheta.norm(); // Magnitude of rotation
      if (theta > 1e-6) { // Avoid division by zero
        Vector3d axis = deltaTheta / theta; // Normalized rotation axis
        Quaterniond deltaQ(Eigen::AngleAxisd(theta, axis)); // Quaternion from axis-angle
        Quaterniond newOrientation = currentOrientation * deltaQ; // Update orientation
        body->setOrientation(newOrientation);
      } else {
        // For small rotations, use first-order approximation
        Quaterniond omegaQuat(0, angularVelocity.x(), angularVelocity.y(), angularVelocity.z());
        Quaterniond qDot = currentOrientation * omegaQuat;
        qDot.coeffs() *= 0.5;
        Quaterniond newOrientation;
        newOrientation.coeffs() = currentOrientation.coeffs() + qDot.coeffs() * dt;
        newOrientation.normalize();
        body->setOrientation(newOrientation);
      }
    }
  }

  // Store calculation data for debugging
  m_lastJacobian = jacobian;
  m_lastGamma = gamma;
  m_lastForces = forces;


  // --- Constraint Stabilization ---
  solvePositionConstraints(
    1e-10,    // epsilon
    100,      // maxIterations
    0.3,     // alpha
    1e-6,    // lambda
    0.05     // maxCorrection
  );

  solveVelocityConstraints(
    1e-10,    // epsilon
    100,      // maxIterations
    0.3,     // alpha
    1e-6,    // lambda
    0.05     // maxCorrection
  );

}

void DynamicSystem::solvePositionConstraints(
    const double epsilon,
    const int maxIterations,
    const double alpha,
    const double lambda,
    const double maxCorrection) {
  if (m_bodies.empty()) return;

  std::vector<Body *> bodies;
  for (const auto &[id, body]: m_bodies) {
    bodies.emplace_back(body.get());
  }

  for (int iter = 0; iter < maxIterations; ++iter) {
    // Compute constraint violations
    VectorXd c(m_constraints.size());
    int constraintIndex = 0;
    for (const auto &constraint: m_constraints) {
      constraint->computePhi(c, constraintIndex);
      constraintIndex++;
    }

    // Check convergence
    double error = c.norm();
    if (error < epsilon) {
      break;
    }

    // Build Jacobian and solve for corrections
    MatrixXd jacobian;
    VectorXd dummy;
    m_solver.buildJacobian(jacobian, dummy, bodies.size() * 6);

    // Use damped least squares to solve for corrections
    MatrixXd JTJ = jacobian.transpose() * jacobian;
    MatrixXd H = JTJ + lambda * MatrixXd::Identity(JTJ.rows(), JTJ.cols());
    VectorXd dx = H.ldlt().solve(jacobian.transpose() * (-c));

    // Apply corrections with relaxation and clamping
    int i = 0;
    for (auto &body: bodies) {
      if (!body->isFixed()) {
        Vector3d correction = alpha * dx.segment<3>(i * 6);

        // Limit correction magnitude
        if (correction.norm() > maxCorrection) {
          correction *= maxCorrection / correction.norm();
        }

        // Apply correction to position
        body->setPosition(body->getPosition() + correction);
      }
      i++;
    }
  }
}

void DynamicSystem::solveVelocityConstraints(
    const double epsilon,
    const int maxIterations,
    const double alpha,
    const double lambda,
    const double maxCorrection) {
  if (m_bodies.empty()) return;

  std::vector<Body *> bodies;
  for (const auto &[id, body]: m_bodies) {
    bodies.emplace_back(body.get());
  }

  for (int iter = 0; iter < maxIterations; ++iter) {
    // Build Jacobian
    MatrixXd jacobian;
    VectorXd dummy;
    m_solver.buildJacobian(jacobian, dummy, bodies.size() * 6);

    // Compute velocity constraint violations
    VectorXd qdot(bodies.size() * 6);
    for (int i = 0; i < bodies.size(); ++i) {
      qdot.segment<3>(i * 6) = bodies[i]->getVelocity();
      qdot.segment<3>(i * 6 + 3) = bodies[i]->getAngularVelocity();
    }

    VectorXd cdot = jacobian * qdot;

    // Check convergence
    if (cdot.norm() < epsilon) {
      break;
    }

    // Use damped least squares to solve for velocity corrections
    MatrixXd JTJ = jacobian.transpose() * jacobian;
    MatrixXd H = JTJ + lambda * MatrixXd::Identity(JTJ.rows(), JTJ.cols());
    VectorXd dv = H.ldlt().solve(jacobian.transpose() * (-cdot));

    // Apply corrections with relaxation and clamping
    int i = 0;
    for (auto &body: bodies) {
      if (!body->isFixed()) {
        Vector3d linearCorrection = alpha * dv.segment<3>(i * 6);
        Vector3d angularCorrection = alpha * dv.segment<3>(i * 6 + 3);

        // Limit correction magnitude
        if (linearCorrection.norm() > maxCorrection) {
          linearCorrection *= maxCorrection / linearCorrection.norm();
        }
        if (angularCorrection.norm() > maxCorrection) {
          angularCorrection *= maxCorrection / angularCorrection.norm();
        }

        // Apply corrections to velocities
        body->setVelocity(body->getVelocity() + linearCorrection);
        body->setAngularVelocity(body->getAngularVelocity() + angularCorrection);
      }
      i++;
    }
  }
}


} // namespace Neutron
