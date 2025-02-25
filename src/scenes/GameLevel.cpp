#include "scenes/GameLevel.h"

#include <imgui.h>
#include <core/Camera.h>
#include <core/InputManager.h>
#include <core/SceneManager.h>
#include <scenes/MainMenu.h>
#include <utils/Logger.h>

#include "utils/OpenGLSetup.h"

bool GameLevel::load() {
  return true;
}

void GameLevel::update(float dt) {
  // Handle camera movement
  m_ctx.input->handleCameraMovement(*m_ctx.camera, dt);

  // Use input manager from context
  if (m_ctx.input->isKeyPressed(GLFW_KEY_ENTER)) {
    // Transition to GameLevel using scene manager from context
    m_ctx.scenes->pushScene(std::make_unique<MainMenu>(m_ctx));
    LOG_DEBUG("Transitioning to GameLevel");
  }
}

void GameLevel::render() {
  ImGui::Begin("GameLevel");
  auto pos = m_ctx.camera->getPosition();
  ImGui::Text("Press ENTER to start %.2f, %.2f, %.2f", pos.x, pos.y, pos.z);
  ImGui::End();
}

void GameLevel::unload() {
}