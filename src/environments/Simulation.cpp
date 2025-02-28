#include "environments/Simulation.h"

#include <core/ImGuiManager.h>
#include <core/InputManager.h>
#include <core/Renderer.h>
#include <core/WindowManager.h>
#include <imgui.h>

#include "utils/Logger.h"
#include "utils/OpenGLSetup.h"

bool Simulation::load() {
  LOG_INFO("Initializing Simulation");
  m_camera.setPosition(glm::vec3(0.0f, 0.0f, 0.0f));
  m_camera.setOrientation(glm::quat(1.0f, 0.0f, 0.0f, 0.0f));
  return true;
}

void Simulation::update(float dt) {
  m_ctx.renderer->drawGrid(m_camera);
  // m_ctx.renderer->drawSky(m_camera);
  handleCameraMovement(dt);
}

void Simulation::render() { showUI(); }

void Simulation::unload() { LOG_INFO("Unloading hardcoded simulation..."); }

void Simulation::showUI() {

  // Use const references for vectors
  const glm::vec3 position = m_camera.getPosition();
  const glm::quat orientation = m_camera.getOrientation();

  const glm::vec3 orientationEuler = eulerAngles(m_camera.getOrientation());

  ImGui::Begin("Camera Debug");
  ImGui::Text("Position: (%.2f, %.2f, %.2f)", position.x, position.y,
              position.z);
  ImGui::Text("Orientation: (%.2f, %.2f, %.2f, %.2f)", orientation.w,
              orientation.x, orientation.y, orientation.z);
  ImGui::Text("Orientation Euler: (%.2f, %.2f, %.2f)",
              orientationEuler.z * 180.0f / glm::pi<float>(),
              orientationEuler.y * 180.0f / glm::pi<float>(),
              orientationEuler.x * 180.0f / glm::pi<float>());

  ImGui::End();
}

enum MoveDirection { FORWARD, BACKWARD, LEFT, RIGHT, UP, DOWN };

void Simulation::handleCameraMovement(float dt) {
  // Movement controls
  if (m_ctx.input->isKeyHeld(GLFW_KEY_W))
    m_camera.processKeyboardInput(FORWARD, dt);
  if (m_ctx.input->isKeyHeld(GLFW_KEY_S))
    m_camera.processKeyboardInput(BACKWARD, dt);
  if (m_ctx.input->isKeyHeld(GLFW_KEY_A))
    m_camera.processKeyboardInput(LEFT, dt);
  if (m_ctx.input->isKeyHeld(GLFW_KEY_D))
    m_camera.processKeyboardInput(RIGHT, dt);
  if (m_ctx.input->isKeyHeld(GLFW_KEY_LEFT_SHIFT))
    m_camera.processKeyboardInput(UP, dt);
  if (m_ctx.input->isKeyHeld(GLFW_KEY_LEFT_CONTROL))
    m_camera.processKeyboardInput(DOWN, dt);

  // Right-click look control
  const bool looking = m_ctx.input->isMouseButtonHeld(GLFW_MOUSE_BUTTON_RIGHT);
  glfwSetInputMode(m_ctx.window->getNativeWindow(), GLFW_CURSOR,
                   looking ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);

  if (looking) {
    double xOffset, yOffset;
    m_ctx.input->getMouseDelta(xOffset, yOffset);
    m_camera.processMouseMovement(static_cast<float>(xOffset),
                                  static_cast<float>(yOffset));
  }
}
