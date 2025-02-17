#include "core/InputManager.h"

InputManager::InputManager(GLFWwindow* window)
    : m_window(window), m_lastMouseX(0.0), m_lastMouseY(0.0), m_mouseDeltaX(0.0), m_mouseDeltaY(0.0) {
    // Initialize last known mouse position
    double xpos, ypos;
    glfwGetCursorPos(m_window, &xpos, &ypos);
    m_lastMouseX = xpos;
    m_lastMouseY = ypos;
}

void InputManager::update(float deltaTime) {
    // Process key bindings
    processKeyBindings(deltaTime);

    // Update mouse movement delta
    double currentX, currentY;
    glfwGetCursorPos(m_window, &currentX, &currentY);

    m_mouseDeltaX = currentX - m_lastMouseX;
    m_mouseDeltaY = currentY - m_lastMouseY;

    m_lastMouseX = currentX;
    m_lastMouseY = currentY;
}

// Query key states
bool InputManager::isKeyPressed(int key) const {
    return glfwGetKey(m_window, key) == GLFW_PRESS;
}

bool InputManager::isKeyReleased(int key) const {
    return glfwGetKey(m_window, key) == GLFW_RELEASE;
}

bool InputManager::isKeyHeld(int key) const {
    return glfwGetKey(m_window, key) == GLFW_REPEAT;
}

// Query mouse states
bool InputManager::isMouseButtonPressed(int button) const {
    return glfwGetMouseButton(m_window, button) == GLFW_PRESS;
}

bool InputManager::isMouseButtonReleased(int button) const {
    return glfwGetMouseButton(m_window, button) == GLFW_RELEASE;
}

bool InputManager::isMouseButtonHeld(int button) const {
    return glfwGetMouseButton(m_window, button) == GLFW_REPEAT;
}

void InputManager::getMousePosition(double& x, double& y) const {
    glfwGetCursorPos(m_window, &x, &y);
}

void InputManager::getMouseDelta(double& dx, double& dy) const {
    dx = m_mouseDeltaX;
    dy = m_mouseDeltaY;
}

// Bind custom actions to keys
void InputManager::bindAction(int key, ActionCallback callback) {
    m_keyBindings[key] = callback;
}

// Execute bound actions
void InputManager::processKeyBindings(float deltaTime) {
    for (const auto& [key, callback] : m_keyBindings) {
        if (isKeyPressed(key)) {
            callback(deltaTime);
        }
    }
}
