#ifndef SIMULATION_H
#define SIMULATION_H

#include <graphics/Renderer.h>

#include "environments/Environment.h"
#include "graphics/Camera.h"
#include "system/SystemManager.h"

class Simulation final : public Environment {
public:
  explicit Simulation(const Context &ctx)
    : Environment(ctx),
      m_systemManager(ctx.renderer->getShaderManager()) {
  }

  bool load() override;
  void update(float dt) override;
  void render() override;
  void unload() override;

private:
  SystemManager m_systemManager;
  Camera m_camera;

  void showUI() const;

  void handleCameraMovement(float dt);
  void handleDefaultInputs() const;
  void showWindowDebug();

  float m_displayedFps = 0.0f;
  float m_fpsUpdateTimer = 0.0f;
  static constexpr float FPS_UPDATE_INTERVAL = 1.0f;
};

#endif // SIMULATION_H
