#ifndef SYSTEMBUILDER_H
#define SYSTEMBUILDER_H
#include "SystemConfiguration.h"

class SystemBuilder {
public:
  SystemBuilder &body(const std::string &name, const glm::vec3 &position, const glm::quat &orientation,
                      const glm::vec3 &size, float mass) {
    SystemConfiguration::Body body;
    body.position = position;
    body.orientation = orientation;
    body.size = size;
    body.mass = mass;
    m_config.addBody(name, body);
    return *this;
  }

  SystemConfiguration build() { return std::move(m_config); }

private:
  SystemConfiguration m_config;
};

#endif // SYSTEMBUILDER_H
