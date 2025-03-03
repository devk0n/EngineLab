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

        // Main Menu Bar
        if (ImGui::BeginMenuBar()) {
            if (ImGui::BeginMenu("File")) {
                if (ImGui::MenuItem("Load Configuration")) {
                    // TODO: Implement load configuration
                }
                if (ImGui::MenuItem("Save Configuration")) {
                    // TODO: Implement save configuration
                }
                if (ImGui::MenuItem("Exit")) {
                    // TODO: Handle exit if needed
                }
                ImGui::EndMenu();
            }
            if (ImGui::BeginMenu("Edit")) {
                ImGui::MenuItem("Undo", "Ctrl+Z");
                ImGui::MenuItem("Redo", "Ctrl+Y");
                ImGui::EndMenu();
            }
            ImGui::EndMenuBar();
        }

        // Split window into two columns: Body List and Body Details
        ImGui::Columns(2, "main_columns");

        // Left Panel: Body List with a search filter
        ImGui::BeginChild("BodyList", ImVec2(0, 0), true);
        ImGui::Text("Bodies");
        ImGui::Separator();

        static char searchBuffer[128] = "";
        ImGui::InputText("Search", searchBuffer, sizeof(searchBuffer));

        for (auto &pair : m_system.bodies()) {
            const std::string &bodyName = pair.first;
            // Apply search filter if text is provided
            if (searchBuffer[0] != '\0') {
                std::string lowerName = bodyName;
                std::string lowerSearch = searchBuffer;
                std::transform(lowerName.begin(), lowerName.end(), lowerName.begin(), ::tolower);
                std::transform(lowerSearch.begin(), lowerSearch.end(), lowerSearch.begin(), ::tolower);
                if (lowerName.find(lowerSearch) == std::string::npos)
                    continue;
            }

            // Selectable list item
            bool isSelected = (m_selectedBodyName == bodyName);
            if (ImGui::Selectable(bodyName.c_str(), isSelected)) {
                m_selectedBodyName = bodyName;
                m_guizmo.setSelectedBody(bodyName);
                m_currentBody = pair.second;
                // Convert quaternion to Euler angles for editing
                m_euler = glm::eulerAngles(m_currentBody.orientation);
                // Copy current name into the editable buffer
                std::strncpy(m_bodyNameBuffer, bodyName.c_str(), sizeof(m_bodyNameBuffer));
                m_bodyNameBuffer[sizeof(m_bodyNameBuffer) - 1] = '\0';
            }

            // Inline Duplicate button
            ImGui::SameLine();
            if (ImGui::SmallButton(("D#" + bodyName).c_str())) {
                duplicateBody(bodyName);
            }
            ImGui::SameLine();
            if (ImGui::SmallButton(("X#" + bodyName).c_str())) {
                if (m_selectedBodyName == bodyName) {
                    m_selectedBodyName = "";
                    m_currentBody = SystemConfiguration::Body();
                }
                m_system.removeBody(bodyName);
            }
        }
        ImGui::EndChild();

        ImGui::NextColumn();

        // Right Panel: Details of the selected body
        ImGui::BeginChild("BodyDetails", ImVec2(0, 0), true);
        ImGui::Text("Body Details");
        ImGui::Separator();
        if (!m_selectedBodyName.empty()) {
            ImGui::InputText("Name", m_bodyNameBuffer, sizeof(m_bodyNameBuffer));
            ImGui::DragFloat3("Position", &m_currentBody.position.x, 0.1f);
            ImGui::DragFloat3("Size", &m_currentBody.size.x, 0.1f);
            ImGui::DragFloat("Mass", &m_currentBody.mass, 0.1f);
            // Edit orientation using Euler angles
            ImGui::DragFloat3("Euler Orientation", &m_euler.x, 0.01f);
            // Update quaternion based on edited Euler angles
            m_currentBody.orientation = glm::quat(m_euler);

            if (ImGui::Button("Apply Changes")) {
                std::string newName(m_bodyNameBuffer);
                // If the name changed, update the key in the configuration
                if (newName != m_selectedBodyName) {
                    // m_system.renameBody(m_selectedBodyName, newName);
                    m_selectedBodyName = newName;
                }
                // m_system.updateBody(m_selectedBodyName, m_currentBody);
            }
        } else {
            ImGui::Text("No body selected");
        }
        ImGui::EndChild();

        ImGui::Columns(1);

        // Bottom area: Add Body button
        if (ImGui::Button("Add Body")) {
            ImGui::OpenPopup("AddBodyPopup");
        }

        // Add Body Popup with fields for a new body
        if (ImGui::BeginPopup("AddBodyPopup")) {
            static char newName[256] = "NewBody";
            static glm::vec3 newPosition{0.0f, 0.0f, 0.0f};
            static glm::vec3 newEuler{0.0f, 0.0f, 0.0f};
            static glm::vec3 newSize{1.0f, 1.0f, 1.0f};
            static float newMass = 1.0f;

            ImGui::InputText("Name", newName, sizeof(newName));
            ImGui::DragFloat3("Position", &newPosition.x, 0.1f);
            ImGui::DragFloat3("Orientation (Euler)", &newEuler.x, 0.01f);
            ImGui::DragFloat3("Size", &newSize.x, 0.1f);
            ImGui::DragFloat("Mass", &newMass, 0.1f);

            if (ImGui::Button("Create")) {
                SystemConfiguration::Body body;
                body.position = newPosition;
                body.size = newSize;
                body.mass = newMass;
                // Convert Euler angles (assumed in radians) to quaternion
                body.orientation = glm::quat(glm::radians(newEuler));
                m_system.addBody(newName, body);
                ImGui::CloseCurrentPopup();

                // Reset fields for next use
                std::strcpy(newName, "NewBody");
                newPosition = glm::vec3(0.0f);
                newEuler = glm::vec3(0.0f);
                newSize = glm::vec3(1.0f);
                newMass = 1.0f;
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel")) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
        }

        ImGui::End();
    }

private:
  SystemConfiguration &m_system;
  SystemGuizmo &m_guizmo;
  std::string m_selectedBodyName;
  SystemConfiguration::Body m_currentBody;
  glm::vec3 m_euler = glm::vec3(0.0f);
  char m_bodyNameBuffer[256];

  // Duplicates a body by creating a copy with a unique name.
  void duplicateBody(const std::string &name) {
    auto it = m_system.bodies().find(name);
    if (it != m_system.bodies().end()) {
      SystemConfiguration::Body duplicate = it->second;
      std::string newName = name + "_copy";
      int copyIndex = 1;
      while (m_system.bodies().find(newName) != m_system.bodies().end()) {
        newName = name + "_copy" + std::to_string(copyIndex++);
      }
      m_system.addBody(newName, duplicate);
    }
  }
};

#endif // SYSTEMEDITOR_H
