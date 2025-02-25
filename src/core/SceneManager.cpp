#include "core/SceneManager.h"

#include "utils/Logger.h"

void SceneManager::pushScene(std::unique_ptr<Scene> scene) {
  if (!scene) {
    LOG_ERROR("Attempted to push a null scene");
    return;
  }

  LOG_DEBUG("Pushing new scene onto stack");

  // Initialize the new scene
  if (!scene->load()) {  // Check if loading succeeded
    LOG_ERROR("Failed to load new scene");
    return;  // Do not push the scene if loading fails
  }

  m_scenes.push(std::move(scene));
  LOG_DEBUG("New scene loaded successfully");
}

void SceneManager::popScene() {
  if (m_scenes.empty()) {
    LOG_WARN("No scenes to pop");
    return;
  }

  // Clean up the current scene
  m_scenes.top()->unload();
  m_scenes.pop();
  LOG_DEBUG("Scene popped from stack");

  // Resume the previous scene (if any)
  if (!m_scenes.empty()) {
    m_scenes.top()->load(); // Reload the previous scene
  }
}

void SceneManager::update(float dt) {
  if (m_scenes.empty()) {
    LOG_WARN("No scenes to update");
    return;
  }

  // Update the current scene
  m_scenes.top()->update(dt);
}

void SceneManager::render() {
  if (m_scenes.empty()) {
    LOG_WARN("No scenes to render");
    return;
  }

  // Render the current scene
  m_scenes.top()->render();
}