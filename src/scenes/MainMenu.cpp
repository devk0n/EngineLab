#include "scenes/MainMenu.h"
#include "core/InputManager.h"
#include "utils/Logger.h"

void MainMenu::load() {
  // Access resources via context
  // m_ctx.renderer->loadTexture("menu_bg", "assets/menu_bg.png");
}

void MainMenu::update(float dt) {
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
}

void MainMenu::unload() {
}
