#include "utils/Logger.h"

#include "core/Application.h"
#include <GLFW/glfw3.h>

Application::Application()
    : m_initialized(false),
      m_windowManager(std::make_unique<WindowManager>("EngineLab")) {}

bool Application::initialize() {

    // Initialize logger
    if (Logger::initialize("log.txt")) {
        Logger::setLogLevel(Logger::Level::Debug);
        LOG_INFO("Logger initialized!");
    } else { return false; }

    if (!m_windowManager->initialize()) { return false; }

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

        // Clear the screen
        glClearColor(0.1, 0.1, 0.1, 1.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Swap buffers
        m_windowManager->swapBuffers();
    }
    LOG_INFO("Application closed!");
}

