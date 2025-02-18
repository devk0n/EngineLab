#ifndef WINDOWMANAGER_H
#define WINDOWMANAGER_H

#include <memory>
#include <string>
#include <GLFW/glfw3.h>

class WindowManager {
public:
  // Construction & Destruction
  explicit WindowManager(std::string title);
  ~WindowManager();

  // Core Interface
  bool initialize();
  static void pollEvents();
  void swapBuffers() const;
  [[nodiscard]] bool shouldClose() const;
  void close() const;
  [[nodiscard]] GLFWwindow* getNativeWindow() const;

private:
  // Window Data
  struct WindowData {
    int width = 0;
    int height = 0;
    std::string title;
    GLFWmonitor* primaryMonitor = nullptr;
  };

  // Initialization Helpers
  void calculateWindowSize(const GLFWvidmode* videoMode);
  void setOpenGLHints() const;
  bool createWindow();
  void centerWindow(GLFWwindow* window, const GLFWvidmode* videoMode) const;
  [[nodiscard]] bool initializeGLAD() const;
  void logDebugInfo(const GLFWvidmode* videoMode) const;

  // Member Variables
  std::unique_ptr<GLFWwindow, void(*)(GLFWwindow*)> m_window;
  WindowData m_data;
};

#endif // WINDOWMANAGER_H
