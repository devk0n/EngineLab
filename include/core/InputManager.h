#ifndef INPUTMANAGER_H
#define INPUTMANAGER_H

#include <array>
#include "utils/OpenGLSetup.h"

#include "Camera.h"

class InputManager {
public:
  InputManager();

  bool initialize(GLFWwindow* window);
  void update();

  void handleCameraMovement(Camera &camera, float dt);

  // Keyboard
  [[nodiscard]] bool isKeyPressed(int key) const;
  [[nodiscard]] bool isKeyHeld(int key) const;
  [[nodiscard]] bool isKeyReleased(int key) const;

  // Mouse
  [[nodiscard]] bool isMouseButtonPressed(int button) const;
  [[nodiscard]] bool isMouseButtonHeld(int button) const;
  [[nodiscard]] bool isMouseButtonReleased(int button) const;

  void getMousePosition(double& xPos, double& yPos) const;
  void getMouseDelta(double& dx, double& dy) const;
  void getScrollDelta(double& xOffset, double& yOffset) const;

private:
  static void scrollCallback(GLFWwindow* window, double xOffset, double yOffset);

  GLFWwindow* m_window;

  // Keyboard state
  std::array<bool, GLFW_KEY_LAST + 1> m_currentKeys;
  std::array<bool, GLFW_KEY_LAST + 1> m_previousKeys;

  // Mouse state
  std::array<bool, GLFW_MOUSE_BUTTON_LAST + 1> m_currentMouseButtons;
  std::array<bool, GLFW_MOUSE_BUTTON_LAST + 1> m_previousMouseButtons;
  double m_currentMouseX, m_currentMouseY;
  double m_previousMouseX, m_previousMouseY;

  // Scroll state
  double m_currentScrollX, m_currentScrollY;
  double m_scrollX, m_scrollY;
};


#endif // INPUTMANAGER_H
