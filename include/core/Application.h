#ifndef APPLICATION_H
#define APPLICATION_H

#include <memory>
#include "core/InputManager.h"
#include "core/WindowManager.h"

class Application {
public:
  Application();

  bool initialize();
  void run();

private:
  bool m_initialized;
  std::unique_ptr<WindowManager> m_windowManager;
  std::unique_ptr<InputManager> m_inputManager;

  double m_lastFrameTime = 0.0;
};

#endif // APPLICATION_H
