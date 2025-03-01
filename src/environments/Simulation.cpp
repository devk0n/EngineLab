#include "environments/Simulation.h"

#include <core/ImGuiManager.h>
#include <core/InputManager.h>
#include <core/Renderer.h>
#include <core/WindowManager.h>
#include <imgui.h>

#include "core/Time.h"
#include "system/SystemBuilder.h"
#include "system/SystemManager.h"
#include "utils/Logger.h"
#include "utils/OpenGLSetup.h"

constexpr glm::vec3 ORANGE = {1.0f, 0.647f, 0.0f};
constexpr glm::vec3 BLUE = {0.0f, 0.647f, 1.0f};
constexpr glm::vec3 LIGHT_GRAY = {0.5f, 0.5f, 0.5f};
constexpr glm::vec3 CYAN = {0.0f, 1.0f, 1.0f};
constexpr glm::vec3 MAGENTA = {1.0f, 0.0f, 1.0f};
constexpr glm::vec3 YELLOW = {1.0f, 1.0f, 0.0f};

bool Simulation::load() {
  LOG_INFO("Initializing Simulation");
  m_camera.setPosition(glm::vec3(10.0f, 8.0f, 6.0f));
  m_camera.lookAt(glm::vec3(0.0f, 0.0f, 0.0f));

  // Add test rigid bodies
  RigidBody Arm_1(glm::vec3(0.0f, 1.0f, 1.0f),
                  glm::quat(glm::vec3(glm::radians(0.0f), // x (rotation around X axis)
                                      glm::radians(0.0f), // y (rotation around Y axis)
                                      glm::radians(90.0f) // z (rotation around Z axis)
                                      )),
                  glm::vec3(2.0f, 0.4f, 0.4f), BLUE);

  Arm_1.addGeometry(glm::vec3(-1.0f, 0.0f, 0.0f));
  Arm_1.addGeometry(glm::vec3(1.0f, 0.0f, 0.0f));
  m_rigidBodies.push_back(Arm_1);

  // Add test rigid bodies
  RigidBody Arm_2(glm::vec3(-3.0f * glm::cos(glm::radians(70.0f)), 2.0f, 1.0f + (3.0f * glm::sin(glm::radians(70.0f)))),
                  glm::quat(glm::vec3(glm::radians(0.0f), // x (rotation around X axis)
                                      glm::radians(-70.0f), // y (rotation around Y axis)
                                      glm::radians(180.0f) // z (rotation around Z axis)
                                      )),
                  glm::vec3(6.0f, 0.3f, 0.3f), ORANGE);

  Arm_2.addGeometry(glm::vec3(-3.0f, 0.0f, 0.0f));
  Arm_2.addGeometry(glm::vec3(3.0f, 0.0f, 0.0f));
  m_rigidBodies.push_back(Arm_2);

  auto m_config =
      SystemBuilder()
          .body("Arm 1", glm::vec3(0.0f, 1.0f, 1.0f), glm::quat(glm::vec3(0.0f, 0.0f, glm::radians(90.0f))),
                glm::vec3(1.0f), 1.0f)
          .body("Arm 2", glm::vec3(-3.0f, 2.0f, 1.0f),
                glm::quat(glm::vec3(0.0f, glm::radians(-70.0f), glm::radians(180.0f))), glm::vec3(1.0f), 1.0f)
          .build();

  return true;
}

void Simulation::update(float dt) {
  m_ctx.renderer->drawGrid(m_camera);
  // m_ctx.renderer->drawSky(m_camera);
  handleCameraMovement(dt);
  handleDefaultInputs();

  // system.render();
}

void Simulation::render() {
  showUI();
  showWindowDebug();

  m_systemManager.update();

  glm::mat4 viewProj = m_camera.getProjectionMatrix() * m_camera.getViewMatrix();

  for (const auto &rb: m_rigidBodies) {
    m_ctx.renderer->drawCube(viewProj, rb.position, rb.orientation, rb.size, rb.color);
    m_ctx.renderer->drawAxes(viewProj, rb.position, rb.orientation, 0.5f);
    // Draw lines for geometries of current rigid body
    for (const auto &g: rb.geometries) {
      // Transform both start (local origin) and end points by the rigid body's transform
      glm::vec3 startPoint = rb.position;
      glm::vec3 endPoint = rb.position + (rb.orientation * g);
      m_ctx.renderer->drawLine(viewProj, startPoint, endPoint, LIGHT_GRAY);
    }
  }
}

void Simulation::unload() { LOG_INFO("Unloading hardcoded simulation..."); }

void Simulation::showUI() {

  // Use const references for vectors
  const glm::vec3 position = m_camera.getPosition();
  const glm::quat orientation = m_camera.getOrientation();

  const glm::vec3 orientationEuler = eulerAngles(m_camera.getOrientation());

  ImGui::Begin("Camera Debug");
  ImGui::Text("Position: (%.2f, %.2f, %.2f)", position.x, position.y, position.z);
  ImGui::Text("Orientation: (%.2f, %.2f, %.2f, %.2f)", orientation.w, orientation.x, orientation.y, orientation.z);
  ImGui::Text("Orientation Euler: (%.2f, %.2f, %.2f)", orientationEuler.z * 180.0f / glm::pi<float>(),
              orientationEuler.y * 180.0f / glm::pi<float>(), orientationEuler.x * 180.0f / glm::pi<float>());

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
  glfwSetInputMode(m_ctx.window->getNativeWindow(), GLFW_CURSOR, looking ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);

  if (looking) {
    double xOffset, yOffset;
    m_ctx.input->getMouseDelta(xOffset, yOffset);
    m_camera.processMouseMovement(static_cast<float>(xOffset), static_cast<float>(yOffset));
  }

  // Handle scroll independently of looking mode
  double xScrollOffset = 0.0, yScrollOffset = 0.0;
  m_ctx.input->getScrollDelta(xScrollOffset, yScrollOffset);
  m_camera.processScroll(static_cast<float>(yScrollOffset)); // Changed to yScrollOffset
}

void Simulation::handleDefaultInputs() {
  if (m_ctx.input->isKeyPressed(GLFW_KEY_ESCAPE)) {
    m_ctx.window->close();
  }
}

void Simulation::showWindowDebug() {

  float currentFps = ImGui::GetIO().Framerate;

  // Initialize displayed FPS on first frame
  if (m_displayedFps == 0.0f) {
    m_displayedFps = currentFps;
  }

  // Update the displayed FPS value once per second
  m_fpsUpdateTimer += ImGui::GetIO().DeltaTime; // Using ImGui's delta time
  if (m_fpsUpdateTimer >= FPS_UPDATE_INTERVAL) {
    m_displayedFps = currentFps;
    m_fpsUpdateTimer = 0.0f;
  }

  float totalSeconds = m_ctx.time->getElapsedTime();
  int hours = static_cast<int>(totalSeconds) / 3600;
  int minutes = (static_cast<int>(totalSeconds) % 3600) / 60;
  int seconds = static_cast<int>(totalSeconds) % 60;
  int milliseconds = static_cast<int>((totalSeconds - std::floor(totalSeconds)) * 1000);

  ImGui::Begin("Window Debug");
  ImGui::Text("FPS: %.0f (%.2f ms)", m_displayedFps, ImGui::GetIO().DeltaTime * 1000.0f);
  ImGui::Text("Elapsed Time: %02d:%02d:%02d.%03d", hours, minutes, seconds, milliseconds);
  ImGui::Text("Delta Time: %.0f µs", m_ctx.time->getDeltaTime() * 1000000.0f);
  ImGui::End();
}
