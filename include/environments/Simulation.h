#ifndef SIMULATION_H
#define SIMULATION_H

#include <DynamicSystem.h>
#include <graphics/Renderer.h>

#include "environments/Environment.h"
#include "graphics/Camera.h"

#include "SystemVisualizer.h"

class Simulation final : public Environment {
public:
  explicit Simulation(const Context &ctx)
    : Environment(ctx), m_systemVisualizer(ctx.renderer->getShaderManager()) {
  }

  void setupDynamics();

  bool load() override;

  void update(float dt) override;
  void render() override;
  void unload() override;

  void showDynamicsData();

private:
  Camera m_camera;
  SystemVisualizer m_systemVisualizer;
  Neutron::DynamicSystem m_system;

  void showUI() const;

  void handleCameraMovement(float dt);
  void showWindowDebug();

  float m_displayedFps = 0.0f;
  float m_fpsUpdateTimer = 0.0f;
  static constexpr float FPS_UPDATE_INTERVAL = 1.0f;

  mutable bool m_run = false;

  mutable double m_deltaEnergy = 0.0;
  mutable double m_totalEnergy = 0.0;  // Total mechanical energy
  mutable double m_kineticEnergy = 0.0; // Kinetic energy
  mutable double m_potentialEnergy = 0.0; // Potential energy
  mutable double m_prevEnergy = 0.0;  // NEW: Previous frame's energy
};

#endif // SIMULATION_H
