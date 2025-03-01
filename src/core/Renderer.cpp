#include "core/Renderer.h"

#include "utils/OpenGLSetup.h"

// Example vertex data for the grid and sky (full-screen quad).
static float gridVertices[] = {-1000.0f, -1000.0f, 0.0f, 1000.0f, -1000.0f, 0.0f,
                               -1000.0f, 1000.0f,  0.0f, 1000.0f, 1000.0f,  0.0f};

// Full-screen quad vertices
float skyVertices[] = {-1.0f, 1.0f, 0.0f, -1.0f, -1.0f, 0.0f, 1.0f, 1.0f, 0.0f, 1.0f, -1.0f, 0.0f};

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
  glDisable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  glFrontFace(GL_CCW);
  glEnable(GL_MULTISAMPLE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Load common shaders.
  if (!m_shaderManager.loadShader("grid", "../assets/shaders/grid.vert", "../assets/shaders/grid.frag")) {
    LOG_ERROR("Failed to load grid shader");
    return false;
  }
  if (!m_shaderManager.loadShader("sky", "../assets/shaders/sky.vert", "../assets/shaders/sky.frag")) {
    LOG_ERROR("Failed to load sky shader");
    return false;
  }

  // Load the line shader in Renderer::initialize()
  if (!m_shaderManager.loadShader("line", "../assets/shaders/line.vert", "../assets/shaders/line.frag")) {
    LOG_ERROR("Failed to load line shader");
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
  glEnable(GL_DEPTH_TEST);
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
  glUniformMatrix4fv(glGetUniformLocation(shader, "u_viewProjection"), 1, GL_FALSE, glm::value_ptr(viewProjection));

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
  glUniformMatrix4fv(glGetUniformLocation(shader, "u_viewProjection"), 1, GL_FALSE, glm::value_ptr(viewProjection));
  glUniform3f(glGetUniformLocation(shader, "u_cameraPos"), camera.getPosition().x, camera.getPosition().y,
              camera.getPosition().z);
  glUniform1f(glGetUniformLocation(shader, "u_time"), static_cast<float>(glfwGetTime()));

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

void Renderer::drawCube(const glm::mat4 &viewProjection, const glm::vec3 &position, const glm::quat &orientation,
                        const glm::vec3 &size, const glm::vec3 &color) {
  unsigned int shader = m_shaderManager.getShader("line");
  glUseProgram(shader);

  glUniformMatrix4fv(glGetUniformLocation(shader, "u_viewProjection"), 1, GL_FALSE, glm::value_ptr(viewProjection));
  glUniform3fv(glGetUniformLocation(shader, "u_color"), 1, glm::value_ptr(color));

  glm::mat4 model =
      glm::translate(glm::mat4(1.0f), position) * glm::mat4_cast(orientation) * glm::scale(glm::mat4(1.0f), size);
  glUniformMatrix4fv(glGetUniformLocation(shader, "u_model"), 1, GL_FALSE, glm::value_ptr(model));

  // Draw a simple wireframe cube
  float vertices[] = {-0.5f, -0.5f, -0.5f, 0.5f,  -0.5f, -0.5f, 0.5f,  -0.5f, -0.5f, 0.5f,  0.5f,  -0.5f,
                      0.5f,  0.5f,  -0.5f, -0.5f, 0.5f,  -0.5f, -0.5f, 0.5f,  -0.5f, -0.5f, -0.5f, -0.5f,

                      -0.5f, -0.5f, 0.5f,  0.5f,  -0.5f, 0.5f,  0.5f,  -0.5f, 0.5f,  0.5f,  0.5f,  0.5f,
                      0.5f,  0.5f,  0.5f,  -0.5f, 0.5f,  0.5f,  -0.5f, 0.5f,  0.5f,  -0.5f, -0.5f, 0.5f,

                      -0.5f, -0.5f, -0.5f, -0.5f, -0.5f, 0.5f,  0.5f,  -0.5f, -0.5f, 0.5f,  -0.5f, 0.5f,
                      0.5f,  0.5f,  -0.5f, 0.5f,  0.5f,  0.5f,  -0.5f, 0.5f,  -0.5f, -0.5f, 0.5f,  0.5f};

  GLuint vao, vbo;
  glGenVertexArrays(1, &vao);
  glGenBuffers(1, &vbo);

  glBindVertexArray(vao);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), static_cast<void *>(nullptr));
  glEnableVertexAttribArray(0);

  glDrawArrays(GL_LINES, 0, 24);

  glDeleteVertexArrays(1, &vao);
  glDeleteBuffers(1, &vbo);
}

void Renderer::drawLine(const glm::mat4 &viewProjection, const glm::vec3 &start, const glm::vec3 &end,
                        const glm::vec3 &color) {
  unsigned int shader = m_shaderManager.getShader("line");
  glUseProgram(shader);

  glm::mat4 model = glm::mat4(1.0f); // Identity matrix
  glUniformMatrix4fv(glGetUniformLocation(shader, "u_viewProjection"), 1, GL_FALSE, glm::value_ptr(viewProjection));
  glUniformMatrix4fv(glGetUniformLocation(shader, "u_model"), 1, GL_FALSE, glm::value_ptr(model));
  glUniform3fv(glGetUniformLocation(shader, "u_color"), 1, glm::value_ptr(color));

  float vertices[] = {start.x, start.y, start.z, end.x, end.y, end.z};

  GLuint vao, vbo;
  glGenVertexArrays(1, &vao);
  glGenBuffers(1, &vbo);

  glBindVertexArray(vao);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *) 0);
  glEnableVertexAttribArray(0);

  glDrawArrays(GL_LINES, 0, 2);

  glDeleteVertexArrays(1, &vao);
  glDeleteBuffers(1, &vbo);
}

void Renderer::drawAxes(const glm::mat4 &viewProjection, const glm::vec3 &position, const glm::quat &orientation,
                        float length) {
  glm::vec3 xAxis = orientation * glm::vec3(length, 0.0f, 0.0f);
  glm::vec3 yAxis = orientation * glm::vec3(0.0f, length, 0.0f);
  glm::vec3 zAxis = orientation * glm::vec3(0.0f, 0.0f, length);

  drawLine(viewProjection, position, position + xAxis, glm::vec3(0.0f, 1.0f, 1.0f)); // Cyan X
  drawLine(viewProjection, position, position + yAxis, glm::vec3(1.0f, 0.0f, 1.0f)); // Magenta Y
  drawLine(viewProjection, position, position + zAxis, glm::vec3(1.0f, 1.0f, 0.0f)); // Yellow Z
}

ShaderManager &Renderer::getShaderManager() { return m_shaderManager; }

bool Renderer::initGrid() {
  glGenVertexArrays(1, &m_gridVAO);
  glBindVertexArray(m_gridVAO);

  glGenBuffers(1, &m_gridVBO);
  glBindBuffer(GL_ARRAY_BUFFER, m_gridVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(gridVertices), gridVertices, GL_STATIC_DRAW);

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), reinterpret_cast<void *>(0));
  glEnableVertexAttribArray(0);

  glBindVertexArray(0);
  return true;
}

bool Renderer::initSky() {
  glGenVertexArrays(1, &m_skyVAO);
  glBindVertexArray(m_skyVAO);

  glGenBuffers(1, &m_skyVBO);
  glBindBuffer(GL_ARRAY_BUFFER, m_skyVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(skyVertices), skyVertices, GL_STATIC_DRAW);

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), reinterpret_cast<void *>(0));
  glEnableVertexAttribArray(0);

  glBindVertexArray(0);
  return true;
}
