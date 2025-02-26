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

  if (!m_shaderManager.loadShader("sky",
    "../assets/shaders/sky.vert",
    "../assets/shaders/sky.frag")) {
    LOG_ERROR("Failed to load sky shader");
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

  // Create SKY VAO/VBO (cube)
  float skyVertices[] = {
      // Front face (Z-)
      -1.0f, -1.0f, -1.0f,
       1.0f, -1.0f, -1.0f,
      -1.0f,  1.0f, -1.0f,
       1.0f,  1.0f, -1.0f,

      // Back face (Z+)
      -1.0f, -1.0f, 1.0f,
      -1.0f,  1.0f, 1.0f,
       1.0f, -1.0f, 1.0f,
       1.0f,  1.0f, 1.0f,

      // Left face (X-)
      -1.0f, -1.0f, -1.0f,
      -1.0f,  1.0f, -1.0f,
      -1.0f, -1.0f,  1.0f,
      -1.0f,  1.0f,  1.0f,

      // Right face (X+)
       1.0f, -1.0f, -1.0f,
       1.0f, -1.0f,  1.0f,
       1.0f,  1.0f, -1.0f,
       1.0f,  1.0f,  1.0f,

      // Top face (Y+)
      -1.0f,  1.0f, -1.0f,
       1.0f,  1.0f, -1.0f,
      -1.0f,  1.0f,  1.0f,
       1.0f,  1.0f,  1.0f,

      // Bottom face (Y-)
      -1.0f, -1.0f, -1.0f,
      -1.0f, -1.0f,  1.0f,
       1.0f, -1.0f, -1.0f,
       1.0f, -1.0f,  1.0f
  };

  // Create element buffer for cube
  unsigned int skyIndices[] = {
      0,1,2, 2,1,3,       // Front
      4,5,6, 6,5,7,       // Back
      8,9,10, 10,9,11,    // Left
      12,13,14, 14,13,15, // Right
      16,17,18, 18,17,19, // Top
      20,21,22, 22,21,23  // Bottom
  };

  // In initialize()
  glGenBuffers(1, &m_skyEBO);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_skyEBO);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(skyIndices), skyIndices, GL_STATIC_DRAW);

  glGenVertexArrays(1, &m_skyVAO);
  glGenBuffers(1, &m_skyVBO);
  glBindVertexArray(m_skyVAO);
  glBindBuffer(GL_ARRAY_BUFFER, m_skyVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(skyVertices), skyVertices, GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), static_cast<void *>(nullptr));

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
  renderSky(camera);

  // Render grid
  renderGrid(camera);
}

void Renderer::renderSky(const Camera& camera) {
  glDepthMask(GL_FALSE);
  glDisable(GL_DEPTH_TEST);

  unsigned int shader = m_shaderManager.getShader("sky");
  glUseProgram(shader);

  // Remove translation from view matrix
  auto view = glm::mat4(glm::mat3(camera.getViewMatrix()));
  glm::mat4 projection = camera.getProjectionMatrix();

  glUniformMatrix4fv(glGetUniformLocation(shader, "view"),
      1, GL_FALSE, glm::value_ptr(view));
  glUniformMatrix4fv(glGetUniformLocation(shader, "projection"),
      1, GL_FALSE, glm::value_ptr(projection));

  // Draw sky cube
  glBindVertexArray(m_skyVAO);
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 24);  // 24 vertices in total
  glBindVertexArray(0);

  glDepthMask(GL_TRUE);
  glEnable(GL_DEPTH_TEST);
}

void Renderer::renderGrid(const Camera &camera) {

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


