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
    // Transition to MainMenu using scene manager from context
    m_ctx.scenes->pushScene(std::make_unique<MainMenu>(m_ctx));
    LOG_DEBUG("Transitioning to MainMenu");
  }
}

void GameLevel::render() {
  ImGui::ShowDemoWindow();
}

void GameLevel::unload() {
}