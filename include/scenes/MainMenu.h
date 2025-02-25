#ifndef MAINMENU_H
#define MAINMENU_H

#include "core/Context.h"
#include "scenes/Scene.h"

class MainMenu final : public Scene {
public:
  explicit MainMenu(const Context& ctx) : Scene(ctx) {}
  bool load() override;
  void update(float dt) override;
  void render() override;
  void unload() override;
};

#endif // MAINMENU_H