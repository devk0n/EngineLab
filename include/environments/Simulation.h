#ifndef SIMULATION_H
#define SIMULATION_H

#include "environments/Environment.h"

class Simulation final : public Environment {
public:
  explicit Simulation(const Context& ctx) : Environment(ctx) {}
  bool load() override;
  void update(float dt) override;
  void render() override;
  void unload() override;
};

#endif // SIMULATION_H
