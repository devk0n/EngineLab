#ifndef SYSTEMVISUALIZER_H
#define SYSTEMVISUALIZER_H

#include "SystemConfiguration.h"
#include "graphics/ShaderManager.h"

#include "utils/Logger.h"

class SystemVisualizer {
public:
  explicit SystemVisualizer(const SystemConfiguration &system, ShaderManager &shaderManager) :
      m_system(system), m_shaderManager(shaderManager) {
    LOG_DEBUG("SystemVisualizer created");
    // Initialize the cube mesh (vertices, VAO, VBO, etc.)
    initializeCube();
  }

  ~SystemVisualizer() {
    LOG_DEBUG("SystemVisualizer destroyed");
    // Clean up the cube mesh (delete VAO, VBO, etc.)
    cleanupCube();
  }

  void render(const glm::mat4 &viewMatrix, const glm::mat4 &projectionMatrix) const {
    m_shaderManager.useShader("cubeShader"); // Assuming you have a shader named "cubeShader"

    for (const auto &[name, body] : m_system.bodies()) {
      // Create a model matrix for the body
      auto modelMatrix = glm::mat4(1.0f);
      modelMatrix = translate(modelMatrix, body.position);
      modelMatrix = modelMatrix * mat4_cast(body.orientation);
      modelMatrix = scale(modelMatrix, body.size);

      // Set the shader uniforms
      m_shaderManager.setUniform("model", modelMatrix);
      m_shaderManager.setUniform("view", viewMatrix);
      m_shaderManager.setUniform("projection", projectionMatrix);
      m_shaderManager.setUniform("color", glm::vec3(1.0f, 0.0f, 0.0f)); // Red color for the cube

      // Draw the cube
      drawCube();
    }
  }

private:
  const SystemConfiguration &m_system;
  ShaderManager &m_shaderManager;
  unsigned int m_cubeVAO, m_cubeVBO, m_cubeEBO;

  void initializeCube() {
    // Define the vertices of a cube
    constexpr float vertices[] = {
      // Positions
      -0.5f, -0.5f, -0.5f,
       0.5f, -0.5f, -0.5f,
       0.5f,  0.5f, -0.5f,
      -0.5f,  0.5f, -0.5f,
      -0.5f, -0.5f,  0.5f,
       0.5f, -0.5f,  0.5f,
       0.5f,  0.5f,  0.5f,
      -0.5f,  0.5f,  0.5f
  };

    // Define the indices for the cube
    const unsigned int indices[] = {
      0, 1, 2, 2, 3, 0,
      4, 5, 6, 6, 7, 4,
      0, 1, 5, 5, 4, 0,
      2, 3, 7, 7, 6, 2,
      0, 3, 7, 7, 4, 0,
      1, 2, 6, 6, 5, 1
  };

    // Generate and bind the VAO and VBO
    glGenVertexArrays(1, &m_cubeVAO);
    glGenBuffers(1, &m_cubeVBO);
    glGenBuffers(1, &m_cubeEBO);

    glBindVertexArray(m_cubeVAO);

    glBindBuffer(GL_ARRAY_BUFFER, m_cubeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_cubeEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    // Set the vertex attribute pointers
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), static_cast<void *>(nullptr));
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);
  }

  void cleanupCube() const {
    glDeleteVertexArrays(1, &m_cubeVAO);
    glDeleteBuffers(1, &m_cubeVBO);
    glDeleteBuffers(1, &m_cubeEBO);
  }

  void drawCube() const {
    glBindVertexArray(m_cubeVAO);
    glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
  }
};

#endif // SYSTEMVISUALIZER_H
