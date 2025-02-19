#include "utils/Logger.h"

#include "core/Application.h"

#include <imgui.h>
#include <GLFW/glfw3.h>

#include "core/Context.h"
#include "scenes/MainMenu.h"

Application::Application()
    : m_initialized(false),
      m_windowManager(std::make_unique<WindowManager>("EngineLab")),
      m_inputManager(std::make_unique<InputManager>()),
      m_renderer(std::make_unique<Renderer>()),
      m_imguiManager(std::make_unique<ImGuiManager>()),
      m_sceneManager(std::make_unique<SceneManager>()) {}

bool Application::initialize() {
    // Initialize logger
    if (!Logger::initialize("log.txt")) { return false; }
    Logger::setLogLevel(Logger::Level::Debug);

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

    // Initialize SceneManager
    m_sceneManager = std::make_unique<SceneManager>();

    // Create a context for scenes to access engine systems
    Context ctx{
        .window = m_windowManager.get(),
        .input = m_inputManager.get(),
        .renderer = m_renderer.get(),
        .scenes = m_sceneManager.get()
    };

    // Push the initial scene (e.g., MainMenu)
    m_sceneManager->pushScene(std::make_unique<MainMenu>(ctx));

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

        // Update the active scene
        m_sceneManager->update(deltaTime);

        // Render
        m_renderer->clearScreen();
        m_sceneManager->render();

        m_imguiManager->beginFrame();

        ImGui::ShowDemoWindow();

        m_renderer->render();
        m_imguiManager->endFrame();

        m_windowManager->swapBuffers();
    }

    LOG_INFO("Application closed!");
}
