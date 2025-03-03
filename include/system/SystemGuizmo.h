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
    : m_system(system),
      m_visualizer(visualizer) {
    setupImGuizmoStyle();
  }

  void render(const glm::mat4 &viewMatrix,
              const glm::mat4 &projectionMatrix) {
    if (m_selectedBody.empty()) return;

    // Find the body by name
    const auto it = m_system.bodies().find(m_selectedBody);
    if (it == m_system.bodies().end()) {
      m_selectedBody.clear(); // Clear selection if not found
      return;
    }

    auto &body = it->second;

    // Get the current transform of the selected body
    auto modelMatrix = glm::mat4(1.0f);
    modelMatrix = translate(modelMatrix, body.position);
    modelMatrix = modelMatrix * mat4_cast(body.orientation);
    modelMatrix = scale(modelMatrix, body.size);

    // Render the gizmo
    ImGuizmo::SetOrthographic(false);
    ImGuizmo::SetRect(0, 0,
      ImGui::GetIO().DisplaySize.x,
      ImGui::GetIO().DisplaySize.y);

    Manipulate(
      value_ptr(viewMatrix),
      value_ptr(projectionMatrix),
      m_currentOperation,
      m_currentMode,
      value_ptr(modelMatrix)
    );

    // Decompose the updated matrix back into position, orientation, and scale
    glm::vec3 translation, scale;
    glm::quat orientation;
    glm::vec3 skew;
    glm::vec4 perspective;
    decompose(modelMatrix,
              scale,
              orientation,
              translation,
              skew,
              perspective);

    // Update the selected body's properties
    body.position = translation;
    body.orientation = orientation;
    body.size = scale;
  }

  void setSelectedBody(const std::string& name) {
    m_selectedBody = name;
  }

  [[nodiscard]] std::string getSelectedBodyName() const {
    return m_selectedBody;
  }

  void setOperation(ImGuizmo::OPERATION operation) {
    m_currentOperation = operation;
  }

  void setMode(ImGuizmo::MODE mode) {
    m_currentMode = mode;
  }

  // Add visualization control methods
  void setShowTrajectories(bool show) { m_showTrajectories = show; }
  void setShowCollisionBounds(bool show) { m_showCollisionBounds = show; }
  void setShowBodyNames(bool show) { m_showBodyNames = show; }


private:
  SystemConfiguration &m_system;
  SystemVisualizer &m_visualizer;
  std::string m_selectedBodyName;
  std::string m_selectedBody;
  bool m_showTrajectories = true;
  bool m_showCollisionBounds = false;
  bool m_showBodyNames = true;

  ImGuizmo::OPERATION m_currentOperation = ImGuizmo::TRANSLATE;
  ImGuizmo::MODE m_currentMode = ImGuizmo::LOCAL;

  static void setupImGuizmoStyle() {
    ImGuizmo::Style& style = ImGuizmo::GetStyle();

    [[maybe_unused]] constexpr float ALPHA_SEMI_TRANSPARENT  = 0.5f; // 50% transparency
    [[maybe_unused]] constexpr float ALPHA_FULLY_OPAQUE      = 1.0f; // 100% visible
    [[maybe_unused]] constexpr float ALPHA_FULLY_TRANSPARENT = 0.0f; // 0% visible (invisible)
    [[maybe_unused]] constexpr float ALPHA_SLIGHT            = 0.8f; // 80% visible
    [[maybe_unused]] constexpr float ALPHA_FAINT             = 0.2f; // 20% visible
    [[maybe_unused]] constexpr float ALPHA_MEDIUM            = 0.5f; // 50% visible
    [[maybe_unused]] constexpr float ALPHA_HEAVY             = 0.7f; // 70% visible
    [[maybe_unused]] constexpr float ALPHA_LIGHT             = 0.3f; // 30% visible

    [[maybe_unused]] constexpr ImVec4       CYAN = {0.0f, 1.0f, 1.0f, ALPHA_HEAVY};
    [[maybe_unused]] constexpr ImVec4    MAGENTA = {1.0f, 0.0f, 1.0f, ALPHA_HEAVY};
    [[maybe_unused]] constexpr ImVec4     YELLOW = {1.0f, 1.0f, 0.0f, ALPHA_HEAVY};

    [[maybe_unused]] constexpr ImVec4      WHITE = {1.0f, 1.0f, 1.0f, ALPHA_FULLY_OPAQUE};
    [[maybe_unused]] constexpr ImVec4 LIGHT_GRAY = {0.8f, 0.8f, 0.8f, ALPHA_MEDIUM};

    style.Colors[ImGuizmo::COLOR::SCALE_LINE] = LIGHT_GRAY;
    style.Colors[ImGuizmo::COLOR::TRANSLATION_LINE] = LIGHT_GRAY;

    // Axis colors
    style.Colors[ImGuizmo::COLOR::DIRECTION_X] = CYAN;
    style.Colors[ImGuizmo::COLOR::DIRECTION_Y] = MAGENTA;
    style.Colors[ImGuizmo::COLOR::DIRECTION_Z] = YELLOW;

    // Plane colors
    style.Colors[ImGuizmo::COLOR::PLANE_X] = CYAN;
    style.Colors[ImGuizmo::COLOR::PLANE_Y] = MAGENTA;
    style.Colors[ImGuizmo::COLOR::PLANE_Z] = YELLOW;

    // Hovered/Active colors
    style.Colors[ImGuizmo::COLOR::SELECTION] = LIGHT_GRAY;
    style.Colors[ImGuizmo::COLOR::ROTATION_USING_FILL] = LIGHT_GRAY;
    style.Colors[ImGuizmo::COLOR::ROTATION_USING_BORDER] = LIGHT_GRAY;

    style.CenterCircleSize = 5.0f;
    style.RotationLineThickness = 3.5f;
    style.ScaleLineThickness = 3.5f;
    style.TranslationLineThickness = 3.5f;
    style.HatchedAxisLineThickness = 1.0f;
    style.RotationOuterLineThickness = 4.0f;
    style.ScaleLineCircleSize = 1.0f;
    style.TranslationLineArrowSize = 0.0f;

    // style.Colors[ImGuizmo::COLOR::HATCHED_AXIS_LINES] = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);

    style.Colors[ImGuizmo::COLOR::TEXT] = WHITE;
    // style.Colors[ImGuizmo::COLOR::TEXT_SHADOW] = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
    // style.Colors[ImGuizmo::COLOR::INACTIVE] = ImVec4(1.0f, 1.0f, 1.0f, 1.0f);
  }
};

#endif // SYSTEMGUIZMO_H
