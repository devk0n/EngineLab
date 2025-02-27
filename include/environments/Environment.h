#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <string>
#include "core/Context.h"

class Environment {
public:
  explicit Environment(const Context& ctx) : m_ctx(ctx) {}
  virtual ~Environment() = default;

  virtual bool load(const std::string& filename) = 0;
  virtual void update(float dt) = 0;
  virtual void render() = 0;
  virtual void save(const std::string& filename) = 0;
  virtual void unload() = 0;

protected:
  const Context& m_ctx;
};

#endif // ENVIRONMENT_H
