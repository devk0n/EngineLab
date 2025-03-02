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

  // Initialize global states, load shaders, and create common VAOs/VBOs.
  bool initialize();

  // Frame management: clear the screen at the beginning of a frame.
  void clearScreen();
  void beginFrame();
  void endFrame();

  // Utility drawing functions that environments can use.
  void drawGrid(const Camera &camera);
  void drawSky(const Camera &camera);

  // Access to the shader manager if needed.
  ShaderManager &getShaderManager();

private:
  ShaderManager m_shaderManager;

  // VAOs and VBOs for common primitives.
  GLuint m_gridVAO, m_gridVBO;
  GLuint m_skyVAO, m_skyVBO;

  // Helper functions for initializing the grid and sky resources.
  bool initGrid();
  bool initSky();
};

#endif // RENDERER_H
