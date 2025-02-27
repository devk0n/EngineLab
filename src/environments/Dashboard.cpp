#include "environments/Dashboard.h"
#include <imgui.h>
#include "core/EnvironmentManager.h"
#include "environments/Simulation.h"
#include "utils/Logger.h"

bool Dashboard::load() {
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
    auto simEnv = std::make_unique<Simulation>(m_ctx);
    if (!simEnv) {
      LOG_ERROR("Failed to create Simulation environment!");
    } else {
      LOG_DEBUG("Switching to Simulation");
      m_ctx.environments->pushEnvironment(std::move(simEnv));
    }
  }

  ImGui::End();
}


void Dashboard::unload() {
  LOG_DEBUG("Dashboard::unload() called on: %p", this);
}
