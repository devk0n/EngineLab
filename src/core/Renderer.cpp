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
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  if (!m_shaderManager.loadShader("grid",
                                  "../assets/shaders/grid.vert",
                                  "../assets/shaders/grid.frag")) {

    LOG_ERROR("Failed to load shader");
    return false;
  }

  // Full-screen quad vertices (NDC)
  float quadVertices[] = {
    -1.0f,  1.0f,  // Top-left
    -1.0f, -1.0f,  // Bottom-left
     1.0f,  1.0f,  // Top-right
     1.0f, -1.0f   // Bottom-right
  };

  // Create grid VAO/VBO
  glGenVertexArrays(1, &m_gridVAO);
  glGenBuffers(1, &m_gridVBO);
  glBindVertexArray(m_gridVAO);
  glBindBuffer(GL_ARRAY_BUFFER, m_gridVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(quadVertices), quadVertices, GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), static_cast<void *>(nullptr));
  glBindVertexArray(0);

  // Enable blending
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

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
  glm::mat4 invVP = glm::inverse(projection * view);
  glm::vec3 cameraPos = camera.getPosition();

  unsigned int shader = m_shaderManager.getShader("grid");
  glUseProgram(shader);

  // Set uniforms
  glUniformMatrix4fv(glGetUniformLocation(shader, "view"),
                     1, GL_FALSE, glm::value_ptr(view));
  glUniformMatrix4fv(glGetUniformLocation(shader, "projection"),
                     1, GL_FALSE, glm::value_ptr(projection));
  glUniformMatrix4fv(glGetUniformLocation(shader, "invVP"),
                     1, GL_FALSE, glm::value_ptr(invVP));
  glUniform3f(glGetUniformLocation(shader, "cameraPos"),
              cameraPos.x, cameraPos.y, cameraPos.z);

  // Draw grid
  glBindVertexArray(m_gridVAO);
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
  glBindVertexArray(0);
}

