#ifndef MAINMENU_H
#define MAINMENU_H

#include "Scene.h"
#include "core/Context.h"

class MainMenu final : public Scene {
public:
  explicit MainMenu(const Context& ctx) : Scene(ctx) {}
  bool load() override;
  void update(float dt) override;
  void render() override;
  void unload() override;

private:
  void showCameraDebug();
};

#endif // MAINMENU_H