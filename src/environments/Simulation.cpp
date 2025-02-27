#include "environments/Simulation.h"

#include <imgui.h>

#include "utils/Logger.h"

bool Simulation::load() {
  LOG_INFO("Initializing Hardcoded Simulation");
  return true;
}

void Simulation::update(float dt) {
}

void Simulation::render() {
  ImGui::ShowDemoWindow();
}

void Simulation::unload() {
  LOG_INFO("Unloading hardcoded simulation...");
}
