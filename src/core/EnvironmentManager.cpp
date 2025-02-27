#include "core/EnvironmentManager.h"
#include "utils/Logger.h"

void EnvironmentManager::pushEnvironment(
    std::unique_ptr<Environment> environment) {
  if (!environment) {
    LOG_ERROR("Attempted to push a null environment");
    return;
  }

  LOG_DEBUG("Pushing new environment. Address: %p", environment.get());
  if (!environment->load()) {
    LOG_ERROR("Failed to load environment");
    return;
  }

  m_environments.push(std::move(environment));
  LOG_DEBUG("Environment pushed. New top: %p", m_environments.top().get());
}
void EnvironmentManager::popEnvironment() {
  if (m_environments.empty()) {
    LOG_WARN("No environments to pop.");
    return;
  }

  LOG_DEBUG("Popping environment: %p", m_environments.top().get());

  m_environments.top()->unload();
  m_environments.pop();

  LOG_DEBUG("Environment popped. New top: %p",
            m_environments.empty() ? nullptr : m_environments.top().get());

  // Ensure new environment is loaded
  if (!m_environments.empty()) {
    m_environments.top()->load();
  }
}

void EnvironmentManager::update(float dt) {
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
