#ifndef SYSTEMVISUALIZER_H
#define SYSTEMVISUALIZER_H

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "SystemConfiguration.h"
#include "utils/OpenGLSetup.h"

class SystemVisualizer {
public:
  explicit SystemVisualizer(const SystemConfiguration& system)
        : m_system(system) {}

  void render(const glm::mat4& view, const glm::mat4& projection) {
    glUseProgram(m_shaderProgram);

    // Set view and projection matrices
    glUniformMatrix4fv(glGetUniformLocation(m_shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(m_shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

    // Render bodies
    for (const auto& [name, body] : m_system.bodies()) {
      renderBody(body);
    }
  }

private:
  SystemConfiguration m_system;

  GLuint m_VAO, m_VBO, m_shaderProgram;

  void renderBody(const SystemConfiguration::Body& body) {
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(body.position.x, body.position.y, body.position.z));
    model = glm::scale(model, glm::vec3(body.size.x, body.size.y, body.size.z));

    glUniformMatrix4fv(glGetUniformLocation(m_shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
    glUniform3f(glGetUniformLocation(m_shaderProgram, "color"), 0.8f, 0.3f, 0.2f);

    // Render a cube (you can replace this with a more complex mesh)
    renderCube();
  }

  void renderCube() {
    // Cube vertices and indices
    // (You can precompute these and store them in a buffer)
    // ...
    glDrawArrays(GL_TRIANGLES, 0, 36); // Example for a cube
  }

};

#endif // SYSTEMVISUALIZER_H
