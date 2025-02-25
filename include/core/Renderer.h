#ifndef RENDERER_H
#define RENDERER_H

#include <memory>

#include "Camera.h"
#include "graphics/Shader.h"
#include "utils/Logger.h"
#include "utils/OpenGLSetup.h"


class Renderer {
public:
  Renderer();
  ~Renderer();

  bool initialize();
  void clearScreen() ;
  void render(const Camera& camera);

private:
  GLuint m_gridVAO, m_gridVBO;
  Shader m_gridShader;
};


#endif // RENDERER_H
