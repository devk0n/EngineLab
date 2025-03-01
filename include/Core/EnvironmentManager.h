#ifndef ENVIRONMENTMANAGER_H
#define ENVIRONMENTMANAGER_H

#include <memory>
#include <stack>
#include "Environments/Environment.h"

class EnvironmentManager {
public:
  void pushEnvironment(std::unique_ptr<Environment> environment);
  void popEnvironment();
  void update(float dt);
  void render();

private:
  std::stack<std::unique_ptr<Environment>> m_environments;
};

#endif // ENVIRONMENTMANAGER_H
