#include "utils/Logger.h"

#include "core/Application.h"
#include <GLFW/glfw3.h>

Application::Application()
    : m_initialized(false),
      m_windowManager(std::make_unique<WindowManager>("EngineLab")),
      m_inputManager(std::make_unique<InputManager>()) {}

bool Application::initialize() {

    // Initialize logger
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

    m_initialized = true;
    return m_initialized;
}

void Application::run() {
    if (!m_initialized) {
        LOG_ERROR("Application not initialized!");
        return;
    }

    LOG_INFO("Application started!");
    while (!m_windowManager->shouldClose()) {
        // Handle input and events
        WindowManager::pollEvents();
        m_inputManager->update();

        // Example usage: Close window on ESC press
        if (m_inputManager->isKeyPressed(GLFW_KEY_ESCAPE)) {
            m_windowManager->close();
            LOG_INFO("Window closed by user");
        }

        // Clear the screen
        glClearColor(0.1, 0.1, 0.1, 1.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Swap buffers
        m_windowManager->swapBuffers();
    }
    LOG_INFO("Application closed!");
}

