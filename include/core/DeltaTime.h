#ifndef TIME_H
#define TIME_H

#include "OpenGLSetup.h"

class DeltaTime {
public:
  DeltaTime() : m_lastFrameTime(0.0), m_deltaTime(0.0), m_elapsedTime(0.0) {}

  void update() {
    const double currentTime = glfwGetTime();
    m_deltaTime = static_cast<float>(currentTime - m_lastFrameTime);
    m_lastFrameTime = currentTime;
    m_elapsedTime += m_deltaTime;
  }

  double getDeltaTime() const { return m_deltaTime; }
  double getElapsedTime() const { return m_elapsedTime; }

private:
  double m_lastFrameTime;
  double m_deltaTime;
  double m_elapsedTime;
};

#endif // TIME_H
