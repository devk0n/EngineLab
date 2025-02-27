#include "environments/Dashboard.h"
#include <core/ImGuiManager.h>
#include <imgui.h>

#include "core/EnvironmentManager.h"
#include "environments/Simulation.h"
#include "utils/Logger.h"

bool Dashboard::load() {
  LOG_INFO("Dashboard loaded");
  return true;
}

void Dashboard::update(float dt) {}

void Dashboard::render() { showUI(); }

void Dashboard::unload() {
  LOG_DEBUG("Dashboard::unload() called on: %p", this);
}

void Dashboard::showUI() {
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
