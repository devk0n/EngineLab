#include "core/Application.h"
#include "utils/Logger.h"
#include <GLFW/glfw3.h>

Application::Application()
    : m_initialized(false),
      m_windowManager(std::make_unique<WindowManager>(1920, 1080, "EngineLab")),
      m_camera(std::make_unique<Camera>()) {
    LOG_DEBUG("Application constructor called");
}

Application::~Application() {
    LOG_DEBUG("Application destructor called");
    // No need to manually delete unique_ptr members
}

bool Application::initialize() {
    LOG_INFO("Initializing application...");

    if (!m_windowManager->initialize()) {
        LOG_ERROR("Failed to initialize WindowManager.");
        return false;
    }

    // Initialize InputManager
    m_inputManager = std::make_unique<InputManager>(m_windowManager->getNativeWindow());

    // Initialize Camera
    m_camera = std::make_unique<Camera>();

    // Set up input bindings
    setupInputBindings();

    LOG_INFO("Application initialized successfully.");
    m_initialized = true;
    return true;
}

void Application::run() {
    if (!m_initialized) {
        LOG_ERROR("Application is not initialized!");
        return;
    }

    LOG_INFO("Starting main loop...");

    float lastFrameTime = 0.0f;

    while (!m_windowManager->shouldClose()) {
        // Calculate delta time
        float currentFrameTime = static_cast<float>(glfwGetTime());
        float deltaTime = currentFrameTime - lastFrameTime;
        lastFrameTime = currentFrameTime;

        // Handle input and events
        m_windowManager->pollEvents();
        m_inputManager->update(deltaTime);

        // Clear the screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Render objects
        // ...

        // Swap buffers
        m_windowManager->swapBuffers();
    }

    LOG_INFO("Exiting main loop.");
}

void Application::setupInputBindings() {
    m_inputManager->bindAction(GLFW_KEY_W, [this](float deltaTime) {
        m_camera->moveForward(deltaTime);
    });

    m_inputManager->bindAction(GLFW_KEY_S, [this](float deltaTime) {
        m_camera->moveBackward(deltaTime);
    });

    m_inputManager->bindAction(GLFW_KEY_A, [this](float deltaTime) {
        m_camera->moveLeft(deltaTime);
    });

    m_inputManager->bindAction(GLFW_KEY_D, [this](float deltaTime) {
        m_camera->moveRight(deltaTime);
    });

    m_inputManager->bindAction(GLFW_KEY_LEFT_SHIFT, [this](float deltaTime) {
        m_camera->moveUp(deltaTime);
    });

    m_inputManager->bindAction(GLFW_KEY_LEFT_CONTROL, [this](float deltaTime) {
        m_camera->moveDown(deltaTime);
    });

    m_inputManager->bindAction(GLFW_KEY_F12, [this](float) {
        // Example: Capture screenshot using OpenGL
        // Renderer::captureScreenshot();
        LOG_INFO("Screenshot captured!");
    });
}

float Application::calculateDeltaTime() {
    static float lastTime = 0.0f;
    float currentTime = static_cast<float>(glfwGetTime());
    float deltaTime = currentTime - lastTime;
    lastTime = currentTime;
    return deltaTime;
}
