#ifndef SYSTEMCONFIGURATION_H
#define SYSTEMCONFIGURATION_H

#include <string>
#include <unordered_map>
#include <vector>

#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"

class SystemConfiguration {
public:
  struct Body {
    glm::vec3 position; // Position in 3D space
    glm::quat orientation; // Orientation as a quaternion
    glm::vec3 size; // Dimensions (width, height, depth)
    float mass; // Mass in kg
  };

  // Core Data Access
  const std::unordered_map<std::string, Body> &bodies() const { return m_bodies; }
  std::unordered_map<std::string, Body> &bodies() { return m_bodies; } // Non-const version

  // Modification Interface
  void addBody(const std::string &name, const Body &body) { m_bodies[name] = body; }
  void removeBody(const std::string &name) { m_bodies.erase(name); }

private:
  std::unordered_map<std::string, Body> m_bodies;
};

#endif // SYSTEMCONFIGURATION_H
