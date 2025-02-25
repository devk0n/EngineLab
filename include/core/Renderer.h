#ifndef RENDERER_H
#define RENDERER_H

#include "glm/gtc/type_ptr.hpp"

#include "Camera.h"
#include "ShaderManager.h"
#include "utils/Logger.h"
#include "utils/OpenGLSetup.h"


class Renderer {
public:
  Renderer();
  ~Renderer();

  bool initialize();
  void clearScreen() ;
  void render(const Camera& camera);
  ShaderManager& getShaderManager();

private:
  ShaderManager m_shaderManager;
};


#endif // RENDERER_H
