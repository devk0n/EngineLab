#include "utils/Logger.h"

#include "core/Application.h"
#include <GLFW/glfw3.h>

Application::Application()
    : m_initialized(false),
      m_windowManager(std::make_unique<WindowManager>("EngineLab")),
      m_inputManager(std::make_unique<InputManager>()) {}

bool Application::initialize() {
    if (Logger::initialize("log.txt")) {
        Logger::setLogLevel(Logger::Level::Debug);
        LOG_INFO("Logger initialized!");
    } else { return false; }

    if (!m_windowManager->initialize()) {
        LOG_ERROR("Failed to initialize window manager");
        return false;
    }

    if (!m_inputManager->initialize(m_windowManager->getNativeWindow())) {
        LOG_ERROR("Failed to initialize input manager");
        return false;
    }

    m_renderer = std::make_unique<Renderer>();
    if (!m_renderer->initialize()) {
        LOG_ERROR("Failed to initialize renderer");
        return false;
    }

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
        m_lastFrameTime = currentTime;

        WindowManager::pollEvents();
        m_inputManager->update();

        if (m_inputManager->isKeyPressed(GLFW_KEY_ESCAPE)) {
            m_windowManager->close();
            LOG_INFO("Window closed by user");
        }

        m_renderer->clearScreen();
        m_renderer->render();

        m_windowManager->swapBuffers();
    }

    LOG_INFO("Application closed!");
}


