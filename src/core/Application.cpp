#include "core/Application.h"

#include "core/Context.h"
#include "environments/Simulation.h"
#include "utils/Logger.h"

Application::Application() :
    m_ctx(), m_initialized(false), m_windowManager(std::make_unique<WindowManager>("EngineLab")),
    m_inputManager(std::make_unique<InputManager>()), m_renderer(std::make_unique<Renderer>()),
    m_imguiManager(std::make_unique<ImGuiManager>()), m_environmentManager(std::make_unique<EnvironmentManager>()),
    m_shaderManager(std::make_unique<ShaderManager>()), m_deltaTime(std::make_unique<DeltaTime>()) {}

Application::~Application() {
  m_imguiManager->shutdown();
  LOG_INFO("Application destroyed");
  m_windowManager.reset();
  glfwTerminate();
  LOG_DEBUG("GLFW terminated");
}

bool Application::initialize() {
  // Initialize window manager
  if (!m_windowManager->initialize()) {
    LOG_ERROR("Failed to initialize window manager");
    return false;
  }

  // Initialize input manager
  if (!m_inputManager->initialize(m_windowManager->getNativeWindow())) {
    LOG_ERROR("Failed to initialize input manager");
    return false;
  }

  // Initialize renderer
  if (!m_renderer->initialize()) {
    LOG_ERROR("Failed to initialize renderer");
    return false;
  }

  // Initialize dear imgui manager
  if (!m_imguiManager->initialize(m_windowManager->getNativeWindow())) {
    LOG_ERROR("Failed to initialize ImGui manager");
    return false;
  }

  // Initialize the member context
  m_ctx.window = m_windowManager.get();
  m_ctx.input = m_inputManager.get();
  m_ctx.renderer = m_renderer.get();
  m_ctx.environments = m_environmentManager.get();
  m_ctx.imgui = m_imguiManager.get();
  m_ctx.time = m_deltaTime.get();

  // Push the initial environment using the member context
  m_environmentManager->pushEnvironment(std::make_unique<Simulation>(m_ctx));

  m_initialized = true;
  return m_initialized;
}

void Application::run() {
  if (!m_initialized) {
    LOG_ERROR("Application not initialized!");
    return;
  }

  LOG_INFO("Application started!");
  m_deltaTime->update();

  while (!m_windowManager->shouldClose()) {
    // Delta Time
    m_deltaTime->update();

    // Update
    WindowManager::pollEvents();
    Renderer::beginFrame();
    m_environmentManager->update(m_deltaTime->getDeltaTime());
    m_inputManager->update();
    Renderer::endFrame();

    // Render
    m_imguiManager->beginFrame();
    m_environmentManager->render();
    m_imguiManager->endFrame();
    m_windowManager->swapBuffers();
  }

  LOG_INFO("Application closed!");
}
