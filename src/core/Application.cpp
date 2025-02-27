#include "core/Application.h"

#include "core/Context.h"
#include "environments/Dashboard.h"
#include "utils/Logger.h"

Application::Application()
    : m_initialized(false),
      m_windowManager(std::make_unique<WindowManager>("EngineLab")),
      m_inputManager(std::make_unique<InputManager>()),
      m_renderer(std::make_unique<Renderer>()),
      m_imguiManager(std::make_unique<ImGuiManager>()),
      m_environmentManager(std::make_unique<EnvironmentManager>()),
      m_shaderManager(std::make_unique<ShaderManager>()) {}

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

    // Create a context for scenes to access engine systems
    Context ctx{
        .window = m_windowManager.get(),
        .input = m_inputManager.get(),
        .renderer = m_renderer.get(),
        .environments = m_environmentManager.get(),
        .imgui = m_imguiManager.get(),
    };

    // Push the initial (e.g., Dashboards)
    m_environmentManager->pushEnvironment(std::make_unique<Dashboard>(ctx));

    m_initialized = true;
    return m_initialized;
}

void Application::run() {
    if (!m_initialized) {
        LOG_ERROR("Application not initialized!");
        return;
    }

    LOG_INFO("Application started!");
    m_lastFrameTime = glfwGetTime();

    while (!m_windowManager->shouldClose()) {
        double currentTime = glfwGetTime();
        float deltaTime = static_cast<float>(currentTime - m_lastFrameTime);
        m_lastFrameTime = currentTime;

        WindowManager::pollEvents();
        m_inputManager->update();

        // Add ESC key check here
        if (m_inputManager->isKeyPressed(GLFW_KEY_ESCAPE)) {
            m_windowManager->close();
        }

        m_environmentManager->update(deltaTime);

        m_renderer->clearScreen();
        // m_renderer->render(*m_camera);

        m_imguiManager->beginFrame();
        m_environmentManager->render();
        m_imguiManager->endFrame();

        m_windowManager->swapBuffers();
    }

    LOG_INFO("Application closed!");
}
