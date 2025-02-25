#include "scenes/MainMenu.h"

#include <imgui.h>
#include <core/InputManager.h>
#include <core/SceneManager.h>

#include "scenes/GameLevel.h"
#include "utils/Logger.h"
#include "utils/OpenGLSetup.h"

bool MainMenu::load() {
  // Access resources via context
  // m_ctx.renderer->loadTexture("menu_bg", "assets/menu_bg.png");
  return true;
}

void MainMenu::update(float dt) {
  // Handle camera movement
  m_ctx.input->handleCameraMovement(*m_ctx.camera, dt);

  // Use input manager from context
  if (m_ctx.input->isKeyPressed(GLFW_KEY_ENTER)) {
    // Transition to GameLevel using scene manager from context
    m_ctx.scenes->pushScene(std::make_unique<GameLevel>(m_ctx));
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



  // In your scene's render function
  ImGui::Begin("Camera Debug");
  ImGui::Text("Position: %.2f, %.2f, %.2f", pos.x, pos.y, pos.z);
  ImGui::SliderFloat("Move Speed", m_ctx.camera->getMovementSpeed(), 1.0f, 20.0f);
  ImGui::SliderFloat("Mouse Sens.", m_ctx.camera->getMouseSensitivity(), 0.01f, 1.0f);
  ImGui::End();

}

void MainMenu::unload() {
}
