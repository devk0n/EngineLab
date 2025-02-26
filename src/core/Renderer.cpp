#include "core/Renderer.h"

#include "utils/OpenGLSetup.h"

float vertices[] = {
  -1000.0f, -1000.0f, 0.0f,  // Bottom-left
  -1000.0f,  1000.0f, 0.0f,  // Top-left
   1000.0f, -1000.0f, 0.0f,  // Bottom-right
   1000.0f,  1000.0f, 0.0f   // Top-right
};

// Full-screen quad vertices
float skyVertices[] = {
  -1.0f,  1.0f, 0.0f,
  -1.0f, -1.0f, 0.0f,
   1.0f,  1.0f, 0.0f,
   1.0f, -1.0f, 0.0f
};

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
  glEnable(GL_MULTISAMPLE);

  if (!m_shaderManager.loadShader("grid",
    "../assets/shaders/grid.vert",
    "../assets/shaders/grid.frag")) {
    LOG_ERROR("Failed to load shader");
    return false;
  }

  if (!m_shaderManager.loadShader("sky",
    "../assets/shaders/sky.vert",
    "../assets/shaders/sky.frag")) {
    LOG_ERROR("Failed to load sky shader");
    return false;
  }

  // Create and upload to VBO
  glGenVertexArrays(1, &m_gridVAO);
  glBindVertexArray(m_gridVAO);

  glGenBuffers(1, &m_gridVBO);
  glBindBuffer(GL_ARRAY_BUFFER, m_gridVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), static_cast<void *>(nullptr));
  glEnableVertexAttribArray(0);

  glBindVertexArray(0);

  // Create and upload to VBO
  glGenVertexArrays(1, &m_skyVAO);
  glBindVertexArray(m_skyVAO);

  glGenBuffers(1, &m_skyVBO);
  glBindBuffer(GL_ARRAY_BUFFER, m_skyVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(skyVertices), skyVertices, GL_STATIC_DRAW);

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), static_cast<void *>(nullptr));
  glEnableVertexAttribArray(0);

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
  // Render sky first
  // renderSky(camera);

  // Render grid
  renderGrid(camera);
}

void Renderer::renderSky(const Camera &camera) {


  // Use sky shader
  unsigned int shader = m_shaderManager.getShader("sky");
  glUseProgram(shader);

  // Set uniforms
  auto view = glm::mat4(glm::mat3(camera.getViewMatrix()));  // Remove translation
  glm::mat4 projection = camera.getProjectionMatrix();
  glm::mat4 viewProjection = projection * view;
  glUniformMatrix4fv(glGetUniformLocation(shader, "u_viewProjection"),
                     1, GL_FALSE, glm::value_ptr(viewProjection));
  glUniform3f(glGetUniformLocation(shader, "u_cameraPos"),
             camera.getPosition().x, camera.getPosition().y, camera.getPosition().z);
  glUniform1f(glGetUniformLocation(shader, "u_time"), glfwGetTime());  // Optional: For animated stars

  // Disable depth writing
  glDisable(GL_DEPTH_TEST);
  glDepthMask(GL_FALSE);

  // Draw sky (full-screen quad or sphere)
  glBindVertexArray(m_skyVAO);
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);  // For a full-screen quad
  glBindVertexArray(0);

  glEnable(GL_DEPTH_TEST);
  glDepthMask(GL_TRUE);
}

void Renderer::renderGrid(const Camera &camera) {
  // Get matrices and camera position
  glm::mat4 view = camera.getViewMatrix();
  glm::mat4 projection = camera.getProjectionMatrix();
  glm::mat4 viewProjection = projection * view;
  glm::vec3 cameraPos = camera.getPosition();

  // Get shader and activate
  unsigned int shader = m_shaderManager.getShader("grid");
  glUseProgram(shader);

  // Set uniforms
  glUniformMatrix4fv(glGetUniformLocation(shader, "u_viewProjection"),
                     1, GL_FALSE, glm::value_ptr(viewProjection));
  glUniform3f(glGetUniformLocation(shader, "u_cameraPos"),
             cameraPos.x, cameraPos.y, cameraPos.z);
  glUniform1f(glGetUniformLocation(shader, "u_scale"), 1.0f);
  // Set fog parameters
  glUniform3f(glGetUniformLocation(shader, "u_fogColor"), 0.2f, 0.2f, 0.2f);
  glUniform1f(glGetUniformLocation(shader, "u_fogStart"), 50.0f);  // Fog starts at 50m
  glUniform1f(glGetUniformLocation(shader, "u_fogEnd"), 250.0f);   // Full fog at 200m

  // Disable backface culling and depth testing
  glDisable(GL_CULL_FACE);
  glDisable(GL_DEPTH_TEST);

  // Draw grid
  glBindVertexArray(m_gridVAO);
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
  glBindVertexArray(0);

  // Re-enable backface culling and depth testing
  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
}


