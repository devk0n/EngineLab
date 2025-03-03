#ifndef SYSTEMCONFIGURATION_H
#define SYSTEMCONFIGURATION_H

#include <string>
#include <unordered_map>
#include <vector>

#include "glm/glm.hpp"
#include "glm/gtc/quaternion.hpp"

struct Body {
  glm::vec3 position;
  glm::quat orientation;
  glm::vec3 size;
  float mass;
  glm::vec4 color;

  Body() : position(0), orientation(1,0,0,0), size(1), mass(1.0f),
          color(0.4f, 0.7f, 1.0f, 1.0f) {}
};

class SystemConfiguration {
public:
  SystemConfiguration() = default;
  SystemConfiguration(const SystemConfiguration&) = default;
  SystemConfiguration& operator=(const SystemConfiguration&) = default;

  // Core Data Access
  const std::unordered_map<std::string, Body> &bodies() const {
    return m_bodies;
  }

  std::unordered_map<std::string, Body> &bodies() { return m_bodies; }

  // Modification Interface
  void addBody(const std::string &name, const Body &body) {
    m_bodies[name] = body;
  }

  void removeBody(const std::string &name) { m_bodies.erase(name); }

  bool bodyExists(const std::string& name) const {
    return m_bodies.contains(name);
  }

  Body& getBody(const std::string& name) {
    return m_bodies.at(name);
  }

  void clear() {
    m_bodies.clear();
  }

private:
  std::unordered_map<std::string, Body> m_bodies;
};

#endif // SYSTEMCONFIGURATION_H
