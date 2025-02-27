#ifndef SIMULATION_H
#define SIMULATION_H

#include "environments/Environment.h"

#include "core/Camera.h"

class Simulation final : public Environment {
public:
  explicit Simulation(const Context &ctx) : Environment(ctx) {}
  bool load() override;
  void update(float dt) override;
  void render() override;
  void unload() override;

private:
  Camera m_camera;

  void showUI();

  void handleCameraMovement(float dt);
};

#endif // SIMULATION_H
