#ifndef CONTEXT_H
#define CONTEXT_H

class WindowManager;
class InputManager;
class Renderer;
class EnvironmentManager;
class ImGuiManager;
class Time;

struct Context {
  WindowManager *window{};
  InputManager *input{};
  Renderer *renderer{};
  EnvironmentManager *environments{};
  ImGuiManager *imgui{};
  Time *time = nullptr;
};

#endif // CONTEXT_H
