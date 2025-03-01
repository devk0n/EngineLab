#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <vector>

class RigidBody {
public:
  // Constructor for easier initialization
  RigidBody(glm::vec3 position, glm::quat orientation, glm::vec3 size = glm::vec3(1.0f),
            glm::vec3 color = glm::vec3(1.0f, 0.125f, 0.322f)) :
      position(position), orientation(orientation), size(size), color(color) {}

  glm::vec3 position = glm::vec3(0.0f);
  glm::quat orientation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
  glm::vec3 velocity = glm::vec3(0.0f);
  glm::vec3 angularVelocity = glm::vec3(0.0f);
  float mass = 1.0f;
  glm::vec3 size = glm::vec3(1.0f); // Default cube size (1x1x1)
  glm::vec3 color = glm::vec3(1.0f, 0.125f, 0.322f);

  void addGeometry(glm::vec3 geometry) { geometries.push_back(geometry); }
  std::vector<glm::vec3> geometries;

  void update(float deltaTime);
};

#endif // RIGIDBODY_H
