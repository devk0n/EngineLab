#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class RigidBody {
public:
  glm::vec3 position = glm::vec3(0.0f);
  glm::quat orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
  glm::vec3 velocity = glm::vec3(0.0f);
  glm::vec3 angularVelocity = glm::vec3(0.0f);
  float mass = 1.0f;

  void update(float deltaTime);
};

#endif // RIGIDBODY_H
