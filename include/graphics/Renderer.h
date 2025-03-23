#ifndef RENDERER_H
#define RENDERER_H

#include "pch.h"

#include "Camera.h"
#include "ShaderManager.h"

class Renderer {
public:
  Renderer();
  ~Renderer();

  // Initialize global states, load shaders, and create common VAOs/VBOs.
  bool initialize();

  // Frame management: clear the screen at the beginning of a frame.
  static void beginFrame();
  static void endFrame();

  // Utility drawing functions that environments can use.
  void drawGrid(const Camera &camera) const;
  void drawSky(const Camera &camera) const;

  // Access to the shader manager if needed.
  ShaderManager &getShaderManager();

private:
  ShaderManager m_shaderManager;

  // VAOs and VBOs for common primitives.
  GLuint m_gridVAO, m_gridVBO;
  GLuint m_skyVAO, m_skyVBO;

  static void clearScreen();

  // Helper functions for initializing the grid and sky resources.
  bool initGrid();
  bool initSky();
};

#endif // RENDERER_H
