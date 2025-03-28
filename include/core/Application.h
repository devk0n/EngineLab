#ifndef APPLICATION_H
#define APPLICATION_H

#include "DeltaTime.h"
#include "EnvironmentManager.h"
#include "ImGuiManager.h"
#include "InputManager.h"
#include "WindowManager.h"

#include "graphics/Renderer.h"
#include "graphics/ShaderManager.h"

class Application {
public:
  Application();
  ~Application();

  bool initialize();
  void run();

private:
  Context m_ctx;

  bool m_initialized;
  std::unique_ptr<WindowManager> m_windowManager;
  std::unique_ptr<InputManager> m_inputManager;
  std::unique_ptr<Renderer> m_renderer;
  std::unique_ptr<ImGuiManager> m_imguiManager;
  std::unique_ptr<EnvironmentManager> m_environmentManager;
  std::unique_ptr<ShaderManager> m_shaderManager;
  std::unique_ptr<DeltaTime> m_deltaTime;

  double m_lastFrameTime = 0.0;
};

#endif // APPLICATION_H
