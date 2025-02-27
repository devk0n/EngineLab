#include "environments/Simulation.h"

#include <core/ImGuiManager.h>
#include <core/InputManager.h>
#include <core/Renderer.h>
#include <core/WindowManager.h>
#include <imgui.h>

#include "utils/Logger.h"
#include "utils/OpenGLSetup.h"

bool Simulation::load() {
  LOG_INFO("Initializing Hardcoded Simulation");
  return true;
}

void Simulation::update(float dt) {
  m_ctx.renderer->drawGrid(m_camera);
  m_ctx.renderer->drawSky(m_camera);
  handleCameraMovement(dt);
}

void Simulation::render() { showUI(); }

void Simulation::unload() { LOG_INFO("Unloading hardcoded simulation..."); }

void Simulation::showUI() {
  // Use const references for vectors
  const glm::vec3 &position = m_camera.getPosition();
  const glm::vec3 &front = m_camera.getFront();
  const glm::vec3 &left = m_camera.getLeft();
  const glm::vec3 &up = m_camera.getUp();

  // Get float values (returned by value, safe)
  float yaw = m_camera.getYaw();
  float pitch = m_camera.getPitch();
  float fov = m_camera.getFov();
  float nearClip = m_camera.getNearClip();
  float farClip = m_camera.getFarClip();
  float moveSpeed = m_camera.getMovementSpeed();
  float mouseSensitivity = m_camera.getMouseSensitivity();

  ImGui::Begin("Camera Debug");

  // Use ImGui Table for better alignment
  if (ImGui::BeginTable("Camera Info", 2,
                        ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
    ImGui::TableSetupColumn("Property", ImGuiTableColumnFlags_WidthFixed,
                            120.0f);
    ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch);
    ImGui::TableHeadersRow();

#define DISPLAY_VECTOR(name, vec)                                              \
  ImGui::TableNextRow();                                                       \
  ImGui::TableNextColumn();                                                    \
  ImGui::Text(name);                                                           \
  ImGui::TableNextColumn();                                                    \
  ImGui::Text("(%.2f, %.2f, %.2f)", vec.x, vec.y, vec.z);

    DISPLAY_VECTOR("Position", position);
    DISPLAY_VECTOR("Front", front);
    DISPLAY_VECTOR("Left", left);
    DISPLAY_VECTOR("Up", up);

#undef DISPLAY_VECTOR

#define DISPLAY_FLOAT(name, value)                                             \
  ImGui::TableNextRow();                                                       \
  ImGui::TableNextColumn();                                                    \
  ImGui::Text(name);                                                           \
  ImGui::TableNextColumn();                                                    \
  ImGui::Text("%.2f", value);

    DISPLAY_FLOAT("Yaw", yaw);
    DISPLAY_FLOAT("Pitch", pitch);
    DISPLAY_FLOAT("FOV", fov);
    DISPLAY_FLOAT("Near Clip", nearClip);
    DISPLAY_FLOAT("Far Clip", farClip);
    DISPLAY_FLOAT("Move Speed", moveSpeed);
    DISPLAY_FLOAT("Mouse Sens.", mouseSensitivity);

#undef DISPLAY_FLOAT

    ImGui::EndTable();
  }

  ImGui::Separator();

  // Editable parameters
  if (ImGui::SliderFloat("Yaw", &yaw, -180.0f, 180.0f)) {
    m_camera.setYaw(yaw);
  }
  if (ImGui::SliderFloat("Pitch", &pitch, -89.0f, 89.0f)) {
    m_camera.setPitch(pitch);
  }
  if (ImGui::SliderFloat("FOV", &fov, 10.0f, 120.0f)) {
    m_camera.setFov(fov);
  }
  if (ImGui::SliderFloat("Near Clip", &nearClip, 0.01f, 1.0f)) {
    m_camera.setNearClip(nearClip);
  }
  if (ImGui::SliderFloat("Far Clip", &farClip, 10.0f, 1000.0f)) {
    m_camera.setFarClip(farClip);
  }
  if (ImGui::SliderFloat("Move Speed", &moveSpeed, 1.0f, 20.0f)) {
    m_camera.setMovementSpeed(moveSpeed);
  }
  if (ImGui::SliderFloat("Mouse Sens.", &mouseSensitivity, 0.01f, 1.0f)) {
    m_camera.setMouseSensitivity(mouseSensitivity);
  }

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
