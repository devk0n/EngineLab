#ifndef CONTEXT_H
#define CONTEXT_H

// Forward declarations to avoid circular includes
class WindowManager;
class InputManager;
class Renderer;
class SceneManager;
class Camera;
class ImGuiManager;

struct Context {
  WindowManager* window;
  InputManager* input;
  Renderer* renderer;
  SceneManager* scenes;
  Camera* camera;
  ImGuiManager* imgui;
};

#endif // CONTEXT_H
