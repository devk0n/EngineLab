#ifndef SYSTEMEDITOR_H
#define SYSTEMEDITOR_H

#include <imgui.h>

#include "SystemConfiguration.h"
#include "SystemGuizmo.h"

class SystemEditor {
public:
  explicit SystemEditor(SystemConfiguration &system, SystemGuizmo &guizmo)
    : m_system(system), m_guizmo(guizmo) {
  }

  void render() {
    ImGui::Begin("System Editor");

    // Add Body Button
    if (ImGui::Button("Add Body")) {
      ImGui::OpenPopup("Add Body");
    }

    // Add Body Popup
    if (ImGui::BeginPopup("Add Body")) {
      static char name[256] = "body";
      static glm::vec3 position{0, 0, 0};
      static glm::quat orientation{1, 0, 0, 0}; // Identity quaternion
      static glm::vec3 size{1, 1, 1};
      static float mass = 1.0f;

      ImGui::InputText("Name", name, sizeof(name));
      ImGui::DragFloat3("Position", &position.x, 0.1f);
      ImGui::DragFloat4("Orientation", &orientation.x, 0.01f, -1.0f, 1.0f);
      ImGui::DragFloat3("Size", &size.x, 0.1f);
      ImGui::DragFloat("Mass", &mass, 0.1f);

      if (ImGui::Button("Create")) {
        SystemConfiguration::Body body;
        body.position = position;
        body.orientation = orientation;
        body.size = size;
        body.mass = mass;
        m_system.addBody(name, body);
        ImGui::CloseCurrentPopup();
      }

      ImGui::EndPopup();
    }

    // List of Bodies
    ImGui::Separator();
    ImGui::Text("Bodies");
    for (auto &[name, body]: m_system.bodies()) {
      // Use non-const accessor
      if (ImGui::TreeNode(name.c_str())) {
        // Select Body for Gizmo Interaction
        if (ImGui::Button("Select")) {
          m_guizmo.setSelectedBody(name); // Pass a non-const pointer
        }

        // Delete Body Button
        if (ImGui::Button("Delete")) {
          if (m_guizmo.getSelectedBodyName() == name) {
            m_guizmo.setSelectedBody(""); // Clear selection
          }
          m_system.removeBody(name);
          ImGui::TreePop();
          break;
        }

        ImGui::TreePop();
      }
    }

    ImGui::End();
  }

private:
  SystemConfiguration &m_system;

  SystemGuizmo &m_guizmo;
};

#endif // SYSTEMEDITOR_H
