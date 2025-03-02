#ifndef SYSTEMVISUALIZER_H
#define SYSTEMVISUALIZER_H

#include "core/ShaderManager.h"
#include "SystemConfiguration.h"

class SystemVisualizer {
public:
  explicit SystemVisualizer(const SystemConfiguration &system, ShaderManager &shaderManager) :
      m_system(system) {}

private:
  const SystemConfiguration &m_system;

};

#endif // SYSTEMVISUALIZER_H
