#ifndef IMGUIMANAGER_H
#define IMGUIMANAGER_H

#include "GLFW/glfw3.h"

class ImGuiManager {
public:
  bool initialize(GLFWwindow* window);
  void beginFrame();
  void endFrame();
  void shutdown();

private:
  bool m_initialized;
};


#endif //IMGUIMANAGER_H
