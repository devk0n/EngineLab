#include "core/InputManager.h"

#include "core/Camera.h"
#include "utils/Logger.h"

InputManager::InputManager()
    : m_window(nullptr),
      m_currentKeys(),
      m_previousKeys(),
      m_currentMouseButtons(),
      m_previousMouseButtons(),
      m_currentMouseX(0.0),
      m_currentMouseY(0.0),
      m_previousMouseX(0.0),
      m_previousMouseY(0.0),
      m_currentScrollX(0.0),
      m_currentScrollY(0.0),
      m_scrollX(0.0),
      m_scrollY(0.0) {}

bool InputManager::initialize(GLFWwindow* window) {
  m_window = window;
  glfwSetScrollCallback(m_window, scrollCallback);
  glfwSetWindowUserPointer(m_window, this);

  m_currentKeys.fill(false);
  m_previousKeys.fill(false);
  m_currentMouseButtons.fill(false);
  m_previousMouseButtons.fill(false);

  glfwGetCursorPos(m_window, &m_currentMouseX, &m_currentMouseY);
  m_previousMouseX = m_currentMouseX;
  m_previousMouseY = m_currentMouseY;

  return true;
}

void InputManager::update() {
    m_previousKeys = m_currentKeys;
    m_previousMouseButtons = m_currentMouseButtons;
    m_previousMouseX = m_currentMouseX;
    m_previousMouseY = m_currentMouseY;

    for (int key = 0; key <= GLFW_KEY_LAST; ++key) {
        m_currentKeys[key] = glfwGetKey(m_window, key) == GLFW_PRESS;
    }

    for (int button = 0; button <= GLFW_MOUSE_BUTTON_LAST; ++button) {
        m_currentMouseButtons[button] = glfwGetMouseButton(m_window, button) == GLFW_PRESS;
    }

    glfwGetCursorPos(m_window, &m_currentMouseX, &m_currentMouseY);

    m_currentScrollX = m_scrollX;
    m_currentScrollY = m_scrollY;
    m_scrollX = 0.0;
    m_scrollY = 0.0;
}

void InputManager::handleCameraMovement(Camera& camera, float dt) {
    // Movement controls
    if (isKeyHeld(GLFW_KEY_W)) camera.processKeyboardInput(CameraMovement::FORWARD, dt);
    if (isKeyHeld(GLFW_KEY_S)) camera.processKeyboardInput(CameraMovement::BACKWARD, dt);
    if (isKeyHeld(GLFW_KEY_A)) camera.processKeyboardInput(CameraMovement::LEFT, dt);
    if (isKeyHeld(GLFW_KEY_D)) camera.processKeyboardInput(CameraMovement::RIGHT, dt);
    if (isKeyHeld(GLFW_KEY_LEFT_SHIFT)) camera.processKeyboardInput(CameraMovement::UP, dt);
    if (isKeyHeld(GLFW_KEY_LEFT_CONTROL)) camera.processKeyboardInput(CameraMovement::DOWN, dt);

    // Right-click look control
    const bool looking = isMouseButtonHeld(GLFW_MOUSE_BUTTON_RIGHT);
    glfwSetInputMode(m_window, GLFW_CURSOR, looking ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);

    if (looking) {
        double xOffset, yOffset;
        getMouseDelta(xOffset, yOffset);
        camera.processMouseMovement(
            static_cast<float>(-xOffset),
            static_cast<float>(-yOffset) // Invert Y axis for natural movement
        );
    }
}


// Keyboard
bool InputManager::isKeyPressed(const int key) const {
    if (key < 0 || key > GLFW_KEY_LAST) return false;
    return m_currentKeys[key] && !m_previousKeys[key];
}

bool InputManager::isKeyHeld(const int key) const {
    if (key < 0 || key > GLFW_KEY_LAST) return false;
    return m_currentKeys[key] && m_previousKeys[key];
}

bool InputManager::isKeyReleased(const int key) const {
    if (key < 0 || key > GLFW_KEY_LAST) return false;
    return !m_currentKeys[key] && m_previousKeys[key];
}

// Mouse
bool InputManager::isMouseButtonPressed(const int button) const {
    if (button < 0 || button > GLFW_MOUSE_BUTTON_LAST) return false;
    return m_currentMouseButtons[button] && !m_previousMouseButtons[button];
}

bool InputManager::isMouseButtonHeld(const int button) const {
    if (button < 0 || button > GLFW_MOUSE_BUTTON_LAST) return false;
    return m_currentMouseButtons[button] && m_previousMouseButtons[button];
}

bool InputManager::isMouseButtonReleased(const int button) const {
    if (button < 0 || button > GLFW_MOUSE_BUTTON_LAST) return false;
    return !m_currentMouseButtons[button] && m_previousMouseButtons[button];
}

void InputManager::getMousePosition(double& xPos, double& yPos) const {
    xPos = m_currentMouseX;
    yPos = m_currentMouseY;
}

void InputManager::getMouseDelta(double& dx, double& dy) const {
    dx = m_currentMouseX - m_previousMouseX;
    dy = m_currentMouseY - m_previousMouseY;
}

void InputManager::getScrollDelta(double& xOffset, double& yOffset) const {
    xOffset = m_currentScrollX;
    yOffset = m_currentScrollY;
}

// Static callback
void InputManager::scrollCallback(GLFWwindow* window, const double xOffset, const double yOffset) {
    if (auto* manager = static_cast<InputManager*>(glfwGetWindowUserPointer(window))) {
        manager->m_scrollX += xOffset;
        manager->m_scrollY += yOffset;
    }
}
