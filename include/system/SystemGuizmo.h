#ifndef SYSTEMGUIZMO_H
#define SYSTEMGUIZMO_H

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "glm/gtx/matrix_decompose.hpp"

#include "ImGuizmo.h"
#include "SystemConfiguration.h"
#include "SystemVisualizer.h"

class SystemGuizmo {
public:
  explicit SystemGuizmo(SystemConfiguration &system,
                        SystemVisualizer &visualizer)
    : m_system(system), m_visualizer(visualizer) {
  }

  void render(const glm::mat4 &viewMatrix,
              const glm::mat4 &projectionMatrix) {
    if (m_selectedBodyName.empty()) return;

    // Find the body by name
    const auto it = m_system.bodies().find(m_selectedBodyName);
    if (it == m_system.bodies().end()) {
      m_selectedBodyName.clear(); // Clear selection if not found
      return;
    }

    auto &body = it->second;

    // Get the current transform of the selected body
    auto modelMatrix = glm::mat4(1.0f);
    modelMatrix = glm::translate(modelMatrix, body.position);
    modelMatrix = modelMatrix * glm::mat4_cast(body.orientation);
    modelMatrix = glm::scale(modelMatrix, body.size);

    // Render the gizmo
    ImGuizmo::SetOrthographic(false);
    ImGuizmo::SetRect(0, 0, ImGui::GetIO().DisplaySize.x,
                      ImGui::GetIO().DisplaySize.y);
    ImGuizmo::Manipulate(
      glm::value_ptr(viewMatrix),
      glm::value_ptr(projectionMatrix),
      m_currentOperation,
      m_currentMode,
      glm::value_ptr(modelMatrix)
    );

    // Decompose the updated matrix back into position, orientation, and scale
    glm::vec3 translation, scale;
    glm::quat orientation;
    glm::vec3 skew;
    glm::vec4 perspective;
    glm::decompose(modelMatrix, scale, orientation, translation, skew,
                   perspective);

    // Update the selected body's properties
    body.position = translation;
    body.orientation = orientation;
    body.size = scale;
  }

  [[nodiscard]] const std::string &getSelectedBodyName() const { return m_selectedBodyName; }

  void setSelectedBody(const std::string &bodyName) {
    m_selectedBodyName = bodyName;
  }

  void setOperation(ImGuizmo::OPERATION operation) {
    m_currentOperation = operation;
  }

  void setMode(ImGuizmo::MODE mode) {
    m_currentMode = mode;
  }

private:
  SystemConfiguration &m_system;
  SystemVisualizer &m_visualizer;
  std::string m_selectedBodyName; // Track by name instead of pointer
  ImGuizmo::OPERATION m_currentOperation = ImGuizmo::TRANSLATE;
  ImGuizmo::MODE m_currentMode = ImGuizmo::LOCAL;
};

#endif // SYSTEMGUIZMO_H
