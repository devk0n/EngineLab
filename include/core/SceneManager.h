#ifndef SCENEMANAGER_H
#define SCENEMANAGER_H

#include <memory>
#include <stack>
#include "scenes/Scene.h"


class SceneManager {
public:
  void pushScene(std::unique_ptr<Scene> scene);
  void popScene();
  void update(float dt);
  void render();

private:
  std::stack<std::unique_ptr<Scene>> m_scenes;
};


#endif //SCENEMANAGER_H
