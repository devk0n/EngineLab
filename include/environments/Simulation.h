#ifndef SIMULATION_H
#define SIMULATION_H

#include "Camera.h"
#include "DynamicSystem.h"
#include "Environment.h"
#include "Renderer.h"
#include "SystemVisualizer.h"

class Simulation final : public Environment {
public:
  explicit Simulation(const Context &ctx)
    : Environment(ctx), m_systemVisualizer(ctx.renderer->getShaderManager()) {
  }

  void initSystem();

  bool load() override;

  void toggle(bool &b);

  void update(float dt) override;
  void render() override;
  void unload() override;

  void showSimulationControls(double dt);

private:
  Camera m_camera;
  SystemVisualizer m_systemVisualizer;
  Neutron::DynamicSystem m_system;

  bool m_run = false;

  void showUI() const;

  void handleCameraMovement(float dt);
  void showWindowDebug();
  float m_displayedFps = 0.0f;
  float m_fpsUpdateTimer = 0.0f;
  static constexpr float FPS_UPDATE_INTERVAL = 1.0f;
};

#endif // SIMULATION_H
