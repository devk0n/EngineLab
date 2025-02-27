#ifndef DASHBOARD_H
#define DASHBOARD_H

#include "environments/Environment.h"

class Dashboard final : public Environment {
public:
  explicit Dashboard(const Context& ctx) : Environment(ctx) {}
  bool load() override;
  void update(float dt) override;
  void render() override;
  void unload() override;

private:
  void showUI(); // UI for selecting environments
};

#endif // DASHBOARD_H
