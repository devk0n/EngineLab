#ifndef SYSTEMMANAGER_H
#define SYSTEMMANAGER_H

#include "SystemConfiguration.h"
#include "SystemEditor.h"

class SystemManager {
public:
  SystemManager() : m_editor(m_system) {}

  void update() {
    m_editor.render();
    // state_manager_.push_state(system_);
  }

  const SystemConfiguration &getSystem() const { return m_system; }

private:
  SystemConfiguration m_system;
  // SystemVisualizer m_visualizer;
  SystemEditor m_editor;
  // SystemStateManager m_stateManager;
};

#endif // SYSTEMMANAGER_H
