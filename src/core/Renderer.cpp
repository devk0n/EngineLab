#include "core/Renderer.h"
#include "GLFW/glfw3.h"

Renderer::Renderer() {}

Renderer::~Renderer() {
  LOG_INFO("Renderer destroyed");
}

bool Renderer::initialize() {
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  glFrontFace(GL_CCW);

  LOG_INFO("Renderer initialized!");
  return true;
}

void Renderer::clearScreen() {
  glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::render(Camera& camera) {

}
