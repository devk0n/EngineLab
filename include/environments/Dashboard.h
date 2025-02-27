#ifndef DASHBOARD_H
#define DASHBOARD_H

#include "environments/Environment.h"

class Dashboard final : public Environment {
public:
  explicit Dashboard(const Context& ctx);
  bool load(const std::string& filename) override;
  void update(float dt) override;
  void render() override;
  void save(const std::string& filename) override;
  void unload() override;

private:
  void showUI(); // UI for selecting environments
};

#endif // DASHBOARD_H
