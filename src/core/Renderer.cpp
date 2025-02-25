#include "core/Renderer.h"

#include "utils/OpenGLSetup.h"

Renderer::Renderer() {}

Renderer::~Renderer() {
  LOG_INFO("Renderer destroyed");
}

bool Renderer::initialize() {
  // Initialize OpenGL settings
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  glFrontFace(GL_CCW);

  if (!m_shaderManager.loadShader("grid",
                                  "../assets/shaders/grid.vert",
                                  "../assets/shaders/grid.frag")) {

    LOG_ERROR("Failed to load shader");
    return false;
  }

  LOG_INFO("Renderer initialized!");
  return true;
}

void Renderer::clearScreen() {
  glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::render(const Camera& camera) {
  auto view = camera.getViewMatrix();
  auto projection = camera.getProjectionMatrix();

  unsigned int shader = m_shaderManager.getShader("grid");
  glUseProgram(shader);

  // Example uniform setup
  glUniformMatrix4fv(glGetUniformLocation(shader, "view"),
                     1,
                     GL_FALSE,
                     glm::value_ptr(view));

  glUniformMatrix4fv(glGetUniformLocation(shader, "projection"),
                     1,
                     GL_FALSE,
                     glm::value_ptr(projection));
}

