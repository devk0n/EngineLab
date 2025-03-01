#include "physics/RigidBody.h"

void RigidBody::update(float deltaTime) {
  // Update position and orientation based on velocity and angular velocity
  position += velocity * deltaTime;
  auto deltaRot = glm::quat(angularVelocity * deltaTime);
  orientation = deltaRot * orientation;
  orientation = normalize(orientation);
}
