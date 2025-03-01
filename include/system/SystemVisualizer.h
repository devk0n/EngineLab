#ifndef SYSTEMVISUALIZER_H
#define SYSTEMVISUALIZER_H

#include "core/ShaderManager.h"

#include "SystemConfiguration.h"
#include "core/Renderer.h"

class SystemVisualizer {
public:
  explicit SystemVisualizer(const SystemConfiguration &system, ShaderManager &shaderManager, Renderer &renderer) :
      m_system(system), m_shaderManager(shaderManager), renderer_(renderer) {}

  void render(const glm::mat4 &viewProjection) {
    // Render bodies
    for (const auto &[name, body]: m_system.bodies()) {
      render_body(body, viewProjection);
    }
  }

private:
  const SystemConfiguration &m_system;
  ShaderManager &m_shaderManager;
  Renderer &m_renderer;

  void render_body(const SystemConfiguration::Body &body, const glm::mat4 &viewProjection) {
    // Draw the body as a cube
    glm::vec3 bodyColor(0.8f, 0.3f, 0.2f); // Orange
    glm::quat bodyOrientation = glm::quat(glm::radians(body.orientation)); // Convert Euler angles to quaternion
    m_renderer.drawCube(viewProjection, body.position, bodyOrientation, body.size, bodyColor);

    // Optionally, draw axes to show orientation
    float axisLength = 1.0f; // Length of the axes
    m_renderer.drawAxes(viewProjection, body.position, bodyOrientation, axisLength);
  }
};

#endif // SYSTEMVISUALIZER_H
