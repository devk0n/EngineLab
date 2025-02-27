#ifndef IMGUIMANAGER_H
#define IMGUIMANAGER_H

#include "utils/OpenGLSetup.h"

class ImGuiManager {
public:
  bool initialize(GLFWwindow* window);
  void beginFrame();
  void endFrame();
  void shutdown();

private:
  bool m_initialized = false;
};


#endif //IMGUIMANAGER_H
