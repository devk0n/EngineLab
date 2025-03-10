#ifndef WINDOWMANAGER_H
#define WINDOWMANAGER_H

#include <memory>
#include <string>

struct WindowData {
  int width = 0;
  int height = 0;
  std::string title;
  GLFWmonitor *primaryMonitor = nullptr;
};

class WindowManager {
public:
  // Construction & Destruction
  explicit WindowManager(std::string title);

  // Core Interface
  bool initialize();
  [[nodiscard]] bool shouldClose() const;

  void swapBuffers() const;
  void close() const;

  static void pollEvents();

  [[nodiscard]] GLFWwindow *getNativeWindow() const;

private:
  // Member Variables
  std::unique_ptr<GLFWwindow, void (*)(GLFWwindow *)> m_window;
  WindowData m_data;

  // Initialization Helpers
  void calculateWindowSize(const GLFWvidmode *videoMode);
  void centerWindow(GLFWwindow *window, const GLFWvidmode *videoMode) const;
  void logDebugInfo(const GLFWvidmode *videoMode) const;

  bool createWindow();

  static bool initializeGLAD();
  static void setOpenGLHints();
};

#endif // WINDOWMANAGER_H
