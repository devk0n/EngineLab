#include "environments/Simulation.h"

#include <DistanceConstraint.h>
#include <DynamicSystem.h>
#include <imgui.h>
#include "InputManager.h"
#include "Logger.h"
#include "NewtonRaphson.h"
#include "OpenGLSetup.h"
#include "Renderer.h"
#include "Time.h"
#include "WindowManager.h"
#include "forces/GravityForceGenerator.h"

constexpr glm::vec3 ORANGE     = {1.0f, 0.647f, 0.0f};
constexpr glm::vec3 BLUE       = {0.0f, 0.647f, 1.0f};
constexpr glm::vec3 LIGHT_GRAY = {0.5f,   0.5f, 0.5f};
constexpr glm::vec3 CYAN       = {0.0f,   1.0f, 1.0f};
constexpr glm::vec3 MAGENTA    = {1.0f,   0.0f, 1.0f};
constexpr glm::vec3 YELLOW     = {1.0f,   1.0f, 0.0f};

using namespace Neutron;

void Simulation::initSystem() {
  // Add particles
  UniqueID particle_1 = m_system.addParticle(1.0, Vector3d(0.0, 0.0, 0.0));
  UniqueID particle_2 = m_system.addParticle(1.0, Vector3d(1.0, 0.0, 0.0));

  // Get particle pointers
  Particle *p1 = m_system.getParticle(particle_1);
  Particle *p2 = m_system.getParticle(particle_2);
  p1->setFixed(true);

  // Add a distance constraint between the particles
  auto constraint1 = std::make_shared<DistanceConstraint>(p1, p2, 2.5);
  m_system.addConstraint(constraint1);

  // Define the constraint function for Newton-Raphson
  auto constraintFunction = [&](const VectorXd& q) -> VectorXd {
    VectorXd c(1); // Single constraint
    constraint1->computeConstraintEquations(c, 0);
    return c;
  };

  // Define the Jacobian function for Newton-Raphson
  auto jacobianFunction = [&](const VectorXd& q) -> MatrixXd {
    MatrixXd jacobian(1, 6); // 1 constraint, 6 DOF (3 for each particle)
    std::map<Particle*, int> particleToIndex = {{p1, 0}, {p2, 1}};
    constraint1->computeJacobian(jacobian, 0, particleToIndex);
    return jacobian;
  };

  // Initial guess for particle positions
  VectorXd initialGuess(6);
  initialGuess << p1->getPosition(), p2->getPosition();

  // Create Newton-Raphson solver
  NewtonRaphson<double, decltype(constraintFunction), decltype(jacobianFunction)> solver;

  // Solve for positions that satisfy the constraint
  try {
    VectorXd solution = solver.solve(
      initialGuess,
      constraintFunction,
      jacobianFunction,
      1e-6, // Convergence tolerance
      100   // Max iterations
    );

    // Update particle positions
    p2->setPosition(solution.segment<3>(3));
    LOG_INFO("Newton-Raphson converged to a valid configuration.");
  } catch (const std::runtime_error& e) {
    LOG_ERROR("Newton-Raphson failed to converge: ", e.what());
  }

  // Add gravity as force generator
  auto gravityGen = std::make_shared<GravityForceGenerator>(Vector3d(0, 0, std::numbers::pi * std::numbers::pi));
  gravityGen->addParticle(p2);
  m_system.addForceGenerator(gravityGen);
}

bool Simulation::load() {

  initSystem();

  LOG_INFO("Initializing Simulation");
  m_camera.setPosition(glm::vec3(10.0f, 8.0f, 4.0f));
  m_camera.lookAt(glm::vec3(0.0f, 0.0f, 0.0f));
  m_camera.setMovementSpeed(10.0f);

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

void Simulation::toggle(bool &b) {
  b = !b;
}

void Simulation::update(const float dt) {

  // m_ctx.renderer->drawSky(m_camera);
  handleCameraMovement(dt);
  if (m_ctx.input->isKeyHeld(GLFW_KEY_SPACE)) {
    m_system.step(dt);
  }
  if (m_ctx.input->isKeyPressed(GLFW_KEY_F1)) {
    toggle(m_run);
  }
  if (m_run) {
    m_system.step(dt);
  }
  if (m_ctx.input->isKeyPressed(GLFW_KEY_L) || m_ctx.input->isKeyHeld(GLFW_KEY_L)) {
    Particle* particle1 = m_system.getParticle(7);
    Vector3d pos1 = particle1->getPosition();
    pos1 += Vector3d(0.0, 0.00001, 0.0);
    particle1->setPosition(pos1);

    Particle* particle2 = m_system.getParticle(16);
    Vector3d pos2 = particle2->getPosition();
    pos2 += Vector3d(0.0, 0.00001, 0.0);
    particle2->setPosition(pos2);
  }
  if (m_ctx.input->isKeyPressed(GLFW_KEY_J) || m_ctx.input->isKeyHeld(GLFW_KEY_J)) {
    Particle* particle1 = m_system.getParticle(7);
    Vector3d pos1 = particle1->getPosition();
    pos1 += Vector3d(0.0, -0.00001, 0.0);
    particle1->setPosition(pos1);

    Particle* particle2 = m_system.getParticle(16);
    Vector3d pos2 = particle2->getPosition();
    pos2 += Vector3d(0.0, -0.00001, 0.0);
    particle2->setPosition(pos2);
  }
}

void Simulation::render() {
  showUI();
  showWindowDebug();
  m_systemVisualizer.render(m_system, m_camera.getPosition(), m_camera.getViewMatrix(), m_camera.getProjectionMatrix());
  m_ctx.renderer->drawGrid(m_camera);
}

void Simulation::unload() { LOG_INFO("Unloading hardcoded simulation..."); }

void Simulation::showSimulationControls(const double dt) {
  ImGui::Begin("Simulation Debug");
  ImGui::Text("Simulation Controls");
  if (!ImGui::Button("Step Simulation")) {
    m_system.step(dt);
  }
  ImGui::End();
}

void Simulation::showUI() const {
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
