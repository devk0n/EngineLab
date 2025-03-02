#ifndef SYSTEMMANAGER_H
#define SYSTEMMANAGER_H

#include "graphics/ShaderManager.h"

#include "SystemConfiguration.h"
#include "SystemEditor.h"
#include "SystemVisualizer.h"

class SystemManager {
public:
  explicit SystemManager(ShaderManager &shaderManager)
    : m_editor(m_system),
      m_visualizer(m_system, shaderManager){}

  void render(const glm::mat4 &viewMatrix, const glm::mat4 &projectionMatrix) {
    m_editor.render();
    m_visualizer.render(viewMatrix, projectionMatrix); // Delegate rendering to SystemVisualizer
  }

  const SystemConfiguration &getSystem() const { return m_system; }

private:
  SystemConfiguration m_system;
  SystemEditor m_editor;
  SystemVisualizer m_visualizer;
  // SystemStateManager m_stateManager;
};

#endif // SYSTEMMANAGER_H
