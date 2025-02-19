#ifndef CONTEXT_H
#define CONTEXT_H

// Forward declarations to avoid circular includes
class WindowManager;
class InputManager;
class Renderer;
class SceneManager;

struct Context {
  WindowManager* window;
  InputManager* input;
  Renderer* renderer;
  SceneManager* scenes;
};

#endif // CONTEXT_H
