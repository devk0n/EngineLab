#ifndef IMGUIMANAGER_H
#define IMGUIMANAGER_H

#include "OpenGLSetup.h"

class ImGuiManager {
public:
  bool initialize(GLFWwindow *window);

  void beginFrame() const;
  void endFrame() const;

  void shutdown();

private:
  bool m_initialized = false;
};

#endif // IMGUIMANAGER_H
