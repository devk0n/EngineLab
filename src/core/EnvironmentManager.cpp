#include "core/EnvironmentManager.h"
#include "utils/Logger.h"

void EnvironmentManager::pushEnvironment(std::unique_ptr<Environment> environment) {
  if (!environment) {
    LOG_ERROR("Attempted to push a null environment");
    return;
  }

  LOG_DEBUG("Pushing new environment onto stack");

  if (!environment->load("default_simulation.json")) {
    LOG_ERROR("Failed to load environment");
    return;
  }

  m_environments.push(std::move(environment));
}

void EnvironmentManager::popEnvironment() {
  if (m_environments.empty()) {
    LOG_WARN("No environments to pop");
    return;
  }

  m_environments.top()->unload();
  m_environments.pop();
  LOG_DEBUG("Environment popped from stack");
}

void EnvironmentManager::update(float dt) {
  if (!m_environments.empty()) {
    m_environments.top()->update(dt);
  }
}

void EnvironmentManager::render() {
  if (!m_environments.empty()) {
    m_environments.top()->render();
  }
}
