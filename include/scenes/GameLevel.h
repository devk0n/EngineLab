#ifndef GAMELEVEL_H
#define GAMELEVEL_H

#include "Scene.h"

class GameLevel final : public Scene {
public:
  explicit GameLevel(const Context& ctx) : Scene(ctx) {}
  bool load() override;
  void update(float dt) override;
  void render() override;
  void unload() override;
};

#endif // GAMELEVEL_H
