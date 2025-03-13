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

  for (auto &generator: m_forceGenerators) { generator->apply(dt); }
  if (m_massInertiaTensor.size() == 0) { buildMassInertiaTensor(); }

  VectorXd forces;
  buildWrench(forces);
  VectorXd accelerations;
  accelerations.resize(forces.size());

  m_solver.solveSystem(
    m_massInertiaTensor,
    forces,
    accelerations
  );

  // --- 2. Update State Variables ---
  for (auto &[id, body]: m_bodies) {
    body->setVelocity(body->getVelocity() + accelerations.segment<3>(id * 6) * dt);
    body->setAngularVelocity(body->getAngularVelocity() + accelerations.segment<3>(id * 6 + 3) * dt);
  }

  // --- 3. Update Position ---
  for (auto &[id, body]: m_bodies) {
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
} // namespace Neutron
