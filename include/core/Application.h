#ifndef APPLICATION_H
#define APPLICATION_H

#include <memory>

#include "Camera.h"
#include "ImGuiManager.h"
#include "ShaderManager.h"
#include "core/InputManager.h"
#include "core/Renderer.h"
#include "core/SceneManager.h"
#include "core/ShaderManager.h"
#include "core/WindowManager.h"

class Application {
public:
  Application();
  ~Application();

  bool initialize();
  void run();

private:
  bool m_initialized;
  std::unique_ptr<WindowManager> m_windowManager;
  std::unique_ptr<InputManager> m_inputManager;
  std::unique_ptr<Renderer> m_renderer;
  std::unique_ptr<ImGuiManager> m_imguiManager;
  std::unique_ptr<Camera> m_camera;
  std::unique_ptr<SceneManager> m_sceneManager;
  std::unique_ptr<ShaderManager> m_shaderManager;

  double m_lastFrameTime = 0.0;
};

#endif // APPLICATION_H
