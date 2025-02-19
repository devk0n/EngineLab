//
// Created by devkon on 19/02/2025.
//

#ifndef MAINMENU_H
#define MAINMENU_H

#include "core/Context.h"
#include "scenes/scene.h"

class MainMenu : public Scene {
public:
  explicit MainMenu(const Context ctx) : Scene(ctx) {}
  void load() override;
  void update(float dt) override;
  void render() override;
  void unload() override;
};


#endif //MAINMENU_H
