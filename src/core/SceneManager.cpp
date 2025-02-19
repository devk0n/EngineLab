#include "core/SceneManager.h"

void SceneManager::pushScene(std::unique_ptr<Scene> scene) {
  m_scenes.push(std::move(scene));
  m_scenes.top()->load();
}

void SceneManager::popScene() {
  if (!m_scenes.empty()) {
    m_scenes.top()->unload();
    m_scenes.pop();
  }
}

void SceneManager::update(float dt) {
  if (!m_scenes.empty()) m_scenes.top()->update(dt);
}

void SceneManager::render() {
  if (!m_scenes.empty()) m_scenes.top()->render();
}