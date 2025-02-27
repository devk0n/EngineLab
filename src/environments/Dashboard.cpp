#include "environments/Dashboard.h"
#include <imgui.h>
#include "core/EnvironmentManager.h"
#include "utils/Logger.h"

Dashboard::Dashboard(const Context& ctx) : Environment(ctx) {}

bool Dashboard::load(const std::string& filename) {
  LOG_INFO("Dashboard loaded");
  return true;
}

void Dashboard::update(float dt) {
  // Transition to Simulation if Enter is pressed
  /*
  if (m_ctx.input->isKeyPressed(GLFW_KEY_ENTER)) {
    // m_ctx.scenes->pushEnvironment(std::make_unique<Simulation>(m_ctx));
    LOG_DEBUG("Switching to Simulation");
  }
*/
}

void Dashboard::render() {
  ImGui::Begin("Dashboard");

  ImGui::Text("Welcome to the Physics Sandbox!");

  if (ImGui::Button("Start Simulation")) {
    // m_ctx.scenes->pushEnvironment(std::make_unique<Simulation>(m_ctx));
  }

  ImGui::End();
}

void Dashboard::save(const std::string& filename) {
  LOG_WARN("Dashboard does not require saving.");
}

void Dashboard::unload() {
  LOG_INFO("Dashboard unloaded");
}