#include "environments/Simulation.h"

#include <core/InputManager.h>
#include <core/WindowManager.h>
#include <imgui.h>

#include "utils/Logger.h"
#include "utils/OpenGLSetup.h"

bool Simulation::load() {
  LOG_INFO("Initializing Hardcoded Simulation");
  return true;
}

void Simulation::update(float dt) { handleCameraMovement(dt); }

void Simulation::render() {
  showCameraDebug();
  ImGui::ShowDemoWindow();
}

void Simulation::unload() { LOG_INFO("Unloading hardcoded simulation..."); }

void Simulation::showCameraDebug() {
  auto position = m_camera.getPosition();
  ImGui::Begin("Camera");
  ImGui::Text("Position: (%f, %f, %f)", position.x, position.y, position.z);
  ImGui::End();
}

void Simulation::handleCameraMovement(float dt) {
  // Movement controls
  if (m_ctx.input->isKeyHeld(GLFW_KEY_W))
    m_camera.processKeyboardInput(CameraMovement::FORWARD, dt);
  if (m_ctx.input->isKeyHeld(GLFW_KEY_S))
    m_camera.processKeyboardInput(CameraMovement::BACKWARD, dt);
  if (m_ctx.input->isKeyHeld(GLFW_KEY_A))
    m_camera.processKeyboardInput(CameraMovement::LEFT, dt);
  if (m_ctx.input->isKeyHeld(GLFW_KEY_D))
    m_camera.processKeyboardInput(CameraMovement::RIGHT, dt);
  if (m_ctx.input->isKeyHeld(GLFW_KEY_LEFT_SHIFT))
    m_camera.processKeyboardInput(CameraMovement::UP, dt);
  if (m_ctx.input->isKeyHeld(GLFW_KEY_LEFT_CONTROL))
    m_camera.processKeyboardInput(CameraMovement::DOWN, dt);

  // Right-click look control
  const bool looking = m_ctx.input->isMouseButtonHeld(GLFW_MOUSE_BUTTON_RIGHT);
  glfwSetInputMode(m_ctx.window->getNativeWindow(), GLFW_CURSOR,
                   looking ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);

  if (looking) {
    double xOffset, yOffset;
    m_ctx.input->getMouseDelta(xOffset, yOffset);
    m_camera.processMouseMovement(static_cast<float>(-xOffset),
                                  static_cast<float>(-yOffset));
  }
}
