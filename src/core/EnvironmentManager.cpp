#include "EnvironmentManager.h"
#include "Logger.h"

void EnvironmentManager::pushEnvironment(
    std::unique_ptr<Environment> environment) {
  if (!environment) {
    LOG_ERROR("Attempted to push a null environment");
    return;
  }

  LOG_DEBUG("Pushing new environment. ");
  if (!environment->load()) {
    LOG_ERROR("Failed to load environment");
    return;
  }

  m_environments.push(std::move(environment));
  LOG_DEBUG("Environment pushed.");
}
void EnvironmentManager::popEnvironment() {
  if (m_environments.empty()) {
    LOG_WARN("No environments to pop.");
    return;
  }

  m_environments.top()->unload();
  m_environments.pop();

  LOG_DEBUG("Environment popped. New top: %p",
            m_environments.empty() ? nullptr : m_environments.top().get());

  // Ensure new environment is loaded
  if (!m_environments.empty()) {
    m_environments.top()->load();
  }
}

void EnvironmentManager::update(const float dt) {
  if (m_environments.empty()) {
    LOG_WARN("No scenes to update");
    return;
  }

  m_environments.top()->update(dt);
}

void EnvironmentManager::render() {
  if (m_environments.empty()) {
    LOG_WARN("No scenes to render");
    return;
  }

  m_environments.top()->render();
}
