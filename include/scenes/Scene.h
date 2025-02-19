#ifndef SCENE_H
#define SCENE_H
#include "core/Context.h"

class Scene {
public:
  explicit Scene(const Context& ctx) : m_ctx(ctx) {}
  virtual ~Scene() = default;
  virtual void load() = 0;     // Load assets
  virtual void update(float dt) = 0;  // Game logic
  virtual void render() = 0;   // Draw to screen
  virtual void unload() = 0;   // Cleanup resources
protected:
  const Context m_ctx;
};

#endif //SCENE_H
