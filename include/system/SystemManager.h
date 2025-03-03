#ifndef SYSTEMMANAGER_H
#define SYSTEMMANAGER_H

#include "graphics/ShaderManager.h"

#include "SystemConfiguration.h"
#include "SystemEditor.h"
#include "SystemVisualizer.h"

class SystemManager {
public:
  explicit SystemManager(ShaderManager &shaderManager)
        : m_editor(m_system, m_guizmo),
          m_visualizer(m_system, shaderManager),
          m_guizmo(m_system, m_visualizer) {}

  void render(const glm::vec3 &cameraPosition, const glm::mat4 &viewMatrix, const glm::mat4 &projectionMatrix) {
    m_editor.render();
    m_visualizer.render(cameraPosition, viewMatrix, projectionMatrix);
    m_guizmo.render(viewMatrix, projectionMatrix); // Render the gizmo
  }

  const SystemConfiguration &getSystem() const { return m_system; }
  SystemGuizmo &getGuizmo() { return m_guizmo; }
private:
  SystemConfiguration m_system;
  SystemEditor m_editor;
  SystemVisualizer m_visualizer;
  SystemGuizmo m_guizmo;
};

#endif // SYSTEMMANAGER_H
