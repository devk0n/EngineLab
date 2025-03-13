#include "environments/Simulation.h"

#include <imgui.h>
#include "DynamicSystem.h"
#include "SphericalJoint.h"
#include "core/InputManager.h"
#include "core/Time.h"
#include "core/WindowManager.h"
#include "forces/GravityForceGenerator.h"
#include "graphics/Renderer.h"
#include "utils/Logger.h"
#include "utils/OpenGLSetup.h"

constexpr glm::vec3 ORANGE     = {1.0f, 0.647f, 0.0f};
constexpr glm::vec3 BLUE       = {0.0f, 0.647f, 1.0f};
constexpr glm::vec3 LIGHT_GRAY = {0.5f,   0.5f, 0.5f};
constexpr glm::vec3 CYAN       = {0.0f,   1.0f, 1.0f};
constexpr glm::vec3 MAGENTA    = {1.0f,   0.0f, 1.0f};
constexpr glm::vec3 YELLOW     = {1.0f,   1.0f, 0.0f};

using namespace Neutron;

void Simulation::setupDynamics() {
  UniqueID body_1 = m_system.addBody(
    10.0,
    Vector3d(10.0, 10.0, 10.0),
    Vector3d(0.0, 0.0, 0.0),
    Quaterniond(1.0, 0.0, 0.0, 0.0)
  );

  UniqueID body_2 = m_system.addBody(
    10.0,
    Vector3d(10.0, 10.0, 10.0),
    Vector3d(10.0, 0.0, 0.0),
    Quaterniond(1.0, 0.0, 0.0, 0.0)
  );

  Body *b1 = m_system.getBody(body_1);
  Body *b2 = m_system.getBody(body_2);

  // Add gravity as force generator
  auto gravityGen = std::make_shared<GravityForceGenerator>(Vector3d(0, 0, -9.81));
  for (auto& body : {b1}) {
    gravityGen->addBody(body);
  }
  m_system.addForceGenerator(gravityGen);
}

bool Simulation::load() {

  setupDynamics();

  LOG_INFO("Initializing Simulation");
  m_camera.setPosition(glm::vec3(12.0f, 20.0f, 20.0f));
  m_camera.lookAt(glm::vec3(0.0f, -1.0f, 0.0f));
  m_camera.setMovementSpeed(5.0f);

  if (!m_ctx.renderer->getShaderManager()
    .loadShader("cubeShader",
                "../assets/shaders/cube.vert",
                "../assets/shaders/cube.frag")) {
    LOG_ERROR("Failed to load cube shader");
    return false;
  }

  if (!m_ctx.renderer->getShaderManager()
    .loadShader("depthShader",
                "../assets/shaders/depth.vert",
                "../assets/shaders/depth.frag")) {
    LOG_ERROR("Failed to load depth shader");
    return false;
  }

  if (!m_ctx.renderer->getShaderManager()
        .loadShader("lineShader",
                    "../assets/shaders/line.vert",
                    "../assets/shaders/line.frag")) {
    LOG_ERROR("Failed to load line shader");
    return false;
  }

  return true;
}

void toggle(bool &value) {
  value = !value;
}

void Simulation::update(const float dt) {
  // m_ctx.renderer->drawSky(m_camera);
  handleCameraMovement(dt);
  if (m_ctx.input->isKeyPressed(GLFW_KEY_SPACE)) { toggle(m_run); }

  if (m_run) {
    m_system.step(dt);
  }
}

void Simulation::render() {
  showUI();
  showWindowDebug();
  m_systemVisualizer.render(m_system, m_camera.getPosition(), m_camera.getViewMatrix(), m_camera.getProjectionMatrix());
  m_ctx.renderer->drawGrid(m_camera);
}

void Simulation::unload() { LOG_INFO("Unloading hardcoded simulation..."); }

void Simulation::showUI() const {
  ImGui::Begin("Energy Debug");
  ImGui::Text("Kinetic: %.3f J", m_kineticEnergy);
  ImGui::Text("Potential: %.3f J", m_potentialEnergy);
  ImGui::Text("Total: %.3f J", m_totalEnergy);
  ImGui::Text("Delta: %.6f J", abs(m_deltaEnergy)); // Show delta energy
  ImGui::End();

  // Use const references for vectors
  const glm::vec3 position = m_camera.getPosition();
  const glm::quat orientation = m_camera.getOrientation();
  const glm::vec3 orientationEuler = eulerAngles(m_camera.getOrientation());

  ImGui::Begin("Camera Debug");
  ImGui::Text("Position: (%.2f, %.2f, %.2f)",
              position.x,
              position.y,
              position.z);
  ImGui::Text("Orientation: (%.2f, %.2f, %.2f, %.2f)",
              orientation.w,
              orientation.x,
              orientation.y,
              orientation.z);
  ImGui::Text("Orientation Euler: (%.2f, %.2f, %.2f)",
              orientationEuler.x * 180.0f / glm::pi<float>(),
              orientationEuler.y * 180.0f / glm::pi<float>(),
              orientationEuler.z * 180.0f / glm::pi<float>());
  ImGui::End();
}

void Simulation::handleCameraMovement(const float dt) {
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
    m_camera.processMouseMovement(static_cast<float>(xOffset),
                                  static_cast<float>(yOffset));
  }

  // Handle scroll independently of looking mode
  double xScrollOffset = 0.0, yScrollOffset = 0.0;
  m_ctx.input->getScrollDelta(xScrollOffset, yScrollOffset);
  m_camera.processScroll(static_cast<float>(yScrollOffset));
  // Changed to yScrollOffset
}

void Simulation::showWindowDebug() {
  const float currentFps = ImGui::GetIO().Framerate;

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

  const double totalSeconds = m_ctx.time->getElapsedTime();
  const int hours = static_cast<int>(totalSeconds) / 3600;
  const int minutes = (static_cast<int>(totalSeconds) % 3600) / 60;
  const int seconds = static_cast<int>(totalSeconds) % 60;
  const int milliseconds = static_cast<int>((totalSeconds - std::floor(totalSeconds)) * 1000);

  ImGui::Begin("Window Debug");
  ImGui::Text("FPS: %.0f (%.2f ms)", m_displayedFps, ImGui::GetIO().DeltaTime * 1000);
  ImGui::Text("Elapsed Time: %02d:%02d:%02d.%03d", hours, minutes, seconds, milliseconds);
  ImGui::Text("Delta Time: %.0f µs", m_ctx.time->getDeltaTime() * 1000000);
  ImGui::End();
}
