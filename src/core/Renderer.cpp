#include "core/Renderer.h"

#include "utils/OpenGLSetup.h"

// Example vertex data for the grid and sky (full-screen quad).
static float gridVertices[] = {-1000.0f, -1000.0f, 0.0f,     1000.0f,
                               -1000.0f, 0.0f,     -1000.0f, 1000.0f,
                               0.0f,     1000.0f,  1000.0f,  0.0f};

// Full-screen quad vertices
float skyVertices[] = {-1.0f, 1.0f, 0.0f, -1.0f, -1.0f, 0.0f,
                       1.0f,  1.0f, 0.0f, 1.0f,  -1.0f, 0.0f};

Renderer::Renderer() : m_gridVAO(0), m_gridVBO(0), m_skyVAO(0), m_skyVBO(0) {}

Renderer::~Renderer() {
  glDeleteVertexArrays(1, &m_gridVAO);
  glDeleteBuffers(1, &m_gridVBO);
  glDeleteVertexArrays(1, &m_skyVAO);
  glDeleteBuffers(1, &m_skyVBO);
  LOG_INFO("Renderer destroyed");
}

bool Renderer::initialize() {
  // Setup global OpenGL state.
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  glFrontFace(GL_CCW);
  glEnable(GL_MULTISAMPLE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Load common shaders.
  if (!m_shaderManager.loadShader("grid", "../assets/shaders/grid.vert",
                                  "../assets/shaders/grid.frag")) {
    LOG_ERROR("Failed to load grid shader");
    return false;
  }
  if (!m_shaderManager.loadShader("sky", "../assets/shaders/sky.vert",
                                  "../assets/shaders/sky.frag")) {
    LOG_ERROR("Failed to load sky shader");
    return false;
  }

  // Initialize resources for grid and sky.
  if (!initGrid() || !initSky()) {
    return false;
  }

  LOG_INFO("Renderer initialized successfully!");
  return true;
}

void Renderer::clearScreen() {
  glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::beginFrame() {
  clearScreen();
  // Additional per-frame setup can be done here.
}

void Renderer::endFrame() {
  // Optionally handle post-processing or other tasks before buffer swap.
}

void Renderer::drawGrid(const Camera &camera) {
  // Use the grid shader.
  unsigned int shader = m_shaderManager.getShader("grid");
  glUseProgram(shader);

  // Compute the view-projection matrix.
  glm::mat4 view = camera.getViewMatrix();
  glm::mat4 projection = camera.getProjectionMatrix();
  glm::mat4 viewProjection = projection * view;
  glUniformMatrix4fv(glGetUniformLocation(shader, "u_viewProjection"), 1,
                     GL_FALSE, glm::value_ptr(viewProjection));

  // Inside Renderer::drawGrid(), after setting the viewProjection uniform
  glUniform3f(glGetUniformLocation(shader, "u_fogColor"), 0.1f, 0.1f, 0.1f);
  glUniform1f(glGetUniformLocation(shader, "u_fogStart"), 50.0f);
  glUniform1f(glGetUniformLocation(shader, "u_fogEnd"), 500.0f);

  // Draw the grid.
  glBindVertexArray(m_gridVAO);
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
  glBindVertexArray(0);
}

void Renderer::drawSky(const Camera &camera) {
  unsigned int shader = m_shaderManager.getShader("sky");
  glUseProgram(shader);

  // Remove translation from the view matrix to keep the sky static.
  glm::mat4 view = glm::mat4(glm::mat3(camera.getViewMatrix()));
  glm::mat4 projection = camera.getProjectionMatrix();
  glm::mat4 viewProjection = projection * view;
  glUniformMatrix4fv(glGetUniformLocation(shader, "u_viewProjection"), 1,
                     GL_FALSE, glm::value_ptr(viewProjection));
  glUniform3f(glGetUniformLocation(shader, "u_cameraPos"),
              camera.getPosition().x, camera.getPosition().y,
              camera.getPosition().z);
  glUniform1f(glGetUniformLocation(shader, "u_time"),
              static_cast<float>(glfwGetTime()));

  // Disable depth testing to ensure the sky is rendered in the background.
  glDisable(GL_DEPTH_TEST);
  glDepthMask(GL_FALSE);

  glBindVertexArray(m_skyVAO);
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
  glBindVertexArray(0);

  // Re-enable depth testing after drawing the sky.
  glEnable(GL_DEPTH_TEST);
  glDepthMask(GL_TRUE);
}

ShaderManager &Renderer::getShaderManager() { return m_shaderManager; }

bool Renderer::initGrid() {
  glGenVertexArrays(1, &m_gridVAO);
  glBindVertexArray(m_gridVAO);

  glGenBuffers(1, &m_gridVBO);
  glBindBuffer(GL_ARRAY_BUFFER, m_gridVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(gridVertices), gridVertices,
               GL_STATIC_DRAW);

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
                        reinterpret_cast<void *>(0));
  glEnableVertexAttribArray(0);

  glBindVertexArray(0);
  return true;
}

bool Renderer::initSky() {
  glGenVertexArrays(1, &m_skyVAO);
  glBindVertexArray(m_skyVAO);

  glGenBuffers(1, &m_skyVBO);
  glBindBuffer(GL_ARRAY_BUFFER, m_skyVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(skyVertices), skyVertices,
               GL_STATIC_DRAW);

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
                        reinterpret_cast<void *>(0));
  glEnableVertexAttribArray(0);

  glBindVertexArray(0);
  return true;
}
