#ifndef INPUTMANAGER_H
#define INPUTMANAGER_H

#include <functional>
#include <unordered_map>
#include <GLFW/glfw3.h>

class InputManager {
public:
  explicit InputManager(GLFWwindow* window);
  ~InputManager() = default;

  void update(float deltaTime);

  // Key input methods
  bool isKeyPressed(int key) const;
  bool isKeyReleased(int key) const;
  bool isKeyHeld(int key) const;

  // Mouse input methods
  bool isMouseButtonPressed(int button) const;
  bool isMouseButtonReleased(int button) const;
  bool isMouseButtonHeld(int button) const;
  void getMousePosition(double& x, double& y) const;
  void getMouseDelta(double& dx, double& dy) const;

  // Bind actions to keys (customizable input mapping)
  using ActionCallback = std::function<void(float)>;
  void bindAction(int key, ActionCallback callback);

private:
  GLFWwindow* m_window;
  double m_lastMouseX, m_lastMouseY;
  double m_mouseDeltaX, m_mouseDeltaY;

  // Stores key bindings
  std::unordered_map<int, ActionCallback> m_keyBindings;

  // Private functions
  void processKeyBindings(float deltaTime);
};

#endif // INPUTMANAGER_H
