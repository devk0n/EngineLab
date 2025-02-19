#include "scenes/MainMenu.h"

#include <imgui.h>
#include <core/ImGuiManager.h>

#include "core/Camera.h"
#include "core/InputManager.h"
#include "utils/Logger.h"

void MainMenu::load() {
  // Access resources via context
  // m_ctx.renderer->loadTexture("menu_bg", "assets/menu_bg.png");
}

void MainMenu::update(float dt) {
  // Handle camera movement
  m_ctx.input->handleCameraMovement(*m_ctx.camera, dt);

  // Use input manager from context
  if (m_ctx.input->isKeyPressed(GLFW_KEY_ENTER)) {
    // Transition to GameLevel using scene manager from context
    // m_ctx.scenes->pushScene(std::make_unique<GameLevel>(m_ctx));
    LOG_DEBUG("Transitioning to GameLevel");
  }
}

void MainMenu::render() {
  // Use renderer from context
  // m_ctx.renderer->DrawTexture("menu_bg", {0, 0});

  ImGui::Begin("MainMenu");
  auto pos = m_ctx.camera->getPosition();
  ImGui::Text("Press ENTER to start %.2f, %.2f, %.2f", pos.x, pos.y, pos.z);
  ImGui::End();

}

void MainMenu::unload() {
}
