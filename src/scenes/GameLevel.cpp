#include "scenes/GameLevel.h"

#include <imgui.h>
#include <core/Camera.h>
#include <core/InputManager.h>
#include <core/SceneManager.h>
#include <scenes/MainMenu.h>
#include <utils/Logger.h>

#include "utils/OpenGLSetup.h"

bool GameLevel::load() {
  m_ctx.camera->setPosition(glm::vec3(10, 8, 6));  // Position above the grid
  m_ctx.camera->lookAt(glm::vec3(0, 0, 0));       // Look at the center of the grid
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