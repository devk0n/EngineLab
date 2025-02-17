#ifndef WINDOWMANAGER_H
#define WINDOWMANAGER_H

#include <memory>
#include <string>
#include <GLFW/glfw3.h>

class WindowManager {
public:
  WindowManager(int width, int height, std::string title);
  ~WindowManager();

  bool initialize();
  void pollEvents() const;
  void swapBuffers() const;
  bool shouldClose() const;
  void close();

  GLFWwindow* getNativeWindow() const { return m_window.get(); }

private:
  std::unique_ptr<GLFWwindow, void(*)(GLFWwindow*)> m_window;
  int m_width;
  int m_height;
  std::string m_title;

  void setGLFWCallbacks();
  static void GLFWErrorCallback(int error, const char* description);
};

#endif // WINDOWMANAGER_H
