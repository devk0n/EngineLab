#ifndef WINDOWMANAGER_H
#define WINDOWMANAGER_H

#include <memory>
#include <string>
#include <GLFW/glfw3.h>

class WindowManager {
public:
  explicit WindowManager(std::string title);

  bool initialize();
  static void pollEvents() ;
  void swapBuffers() const;
  [[nodiscard]] bool shouldClose() const;
  void close() const;

  [[nodiscard]] GLFWwindow* getNativeWindow() const { return m_window.get(); }

private:
  std::unique_ptr<GLFWwindow, void(*)(GLFWwindow*)> m_window;
  int m_width;
  int m_height;
  std::string m_title;

};

#endif // WINDOWMANAGER_H
