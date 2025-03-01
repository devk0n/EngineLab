#ifndef SIMULATION_H
#define SIMULATION_H

#include "environments/Environment.h"

#include "core/Camera.h"
#include "physics/RigidBody.h"

#include <system/SystemManager.h>
#include <vector>

class Simulation final : public Environment {
public:
  explicit Simulation(const Context &ctx) : Environment(ctx) {}
  bool load() override;
  void update(float dt) override;
  void render() override;
  void unload() override;

private:
  SystemManager m_systemManager;

  Camera m_camera;

  std::vector<RigidBody> m_rigidBodies;

  void showUI();

  void handleCameraMovement(float dt);
  void handleDefaultInputs();
  void showWindowDebug();

  float m_displayedFps = 0.0f;
  float m_fpsUpdateTimer = 0.0f;
  static constexpr float FPS_UPDATE_INTERVAL = 1.0f;
};

#endif // SIMULATION_H
