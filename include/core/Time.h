#ifndef TIME_H
#define TIME_H

#include <GLFW/glfw3.h>

class Time {
public:
  Time() : m_lastFrameTime(0.0), m_deltaTime(0.0), m_elapsedTime(0.0) {}

  void update() {
    const double currentTime = glfwGetTime();
    m_deltaTime = currentTime - m_lastFrameTime;
    m_lastFrameTime = currentTime;
    m_elapsedTime += m_deltaTime;
  }

  [[nodiscard]] double getDeltaTime() const { return m_deltaTime; }
  [[nodiscard]] double getElapsedTime() const { return m_elapsedTime; }

private:
  double m_lastFrameTime;
  double m_deltaTime;
  double m_elapsedTime;
};

#endif // TIME_H
