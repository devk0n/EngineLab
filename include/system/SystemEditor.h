#ifndef SYSTEMEDITOR_H
#define SYSTEMEDITOR_H

#include <imgui.h>
#include <stack>

#include "SystemConfiguration.h"
#include "SystemGuizmo.h"

class SystemEditor {
public:
  explicit SystemEditor(SystemConfiguration &system, SystemGuizmo &guizmo)
    : m_system(system), m_guizmo(guizmo) {
    generateDefaultName();
  }

  void render() {
    ImGui::SetNextWindowSize(ImVec2(600, 500), ImGuiCond_FirstUseEver);
    ImGui::Begin("System Editor", nullptr, ImGuiWindowFlags_NoScrollbar);

    // Menu Bar
    if (ImGui::BeginMenuBar()) {
      if (ImGui::BeginMenu("File")) {
        if (ImGui::MenuItem("New")) resetSystem();
        ImGui::EndMenu();
      }
      ImGui::EndMenuBar();
    }

    // Main content area
    ImVec2 contentSize = ImGui::GetContentRegionAvail();
    ImGui::BeginChild("MainContent", ImVec2(contentSize.x * 0.3f, 0), true);

    renderBodyList();

    ImGui::EndChild();
    ImGui::SameLine();
    ImGui::BeginChild("PropertiesPanel", ImVec2(0, 0), true);

    if (!m_selectedBody.empty()) {
      renderBodyProperties();
      m_guizmo.setSelectedBody(m_selectedBody);
    } else {
      ImGui::Text("Select a body to edit");
    }

    ImGui::EndChild();

    ImGui::End();
  }

private:
  SystemConfiguration &m_system;

  SystemGuizmo &m_guizmo;

  std::string m_selectedBody;

  char m_newBodyName[256] = "";

  int m_bodyCounter = 1;

  void generateDefaultName() {
    do {
      std::stringstream ss;
      ss << "Body " << m_bodyCounter++;
      strcpy(m_newBodyName, ss.str().c_str());
    } while (m_system.bodyExists(m_newBodyName));
  }

  void renderBodyList() {
    // Add Body Button
    if (ImGui::Button("+ Add Body")) {
      ImGui::OpenPopup("Add Body");
      generateDefaultName();
    }

    // Add Body Popup
    if (ImGui::BeginPopupModal("Add Body", nullptr,
                               ImGuiWindowFlags_AlwaysAutoResize)) {
      static glm::vec3 newPosition{0};
      static glm::quat newOrientation{1, 0, 0, 0};
      static glm::vec3 newSize{1};
      static float newMass = 1.0f;
      static glm::vec4 newColor{0.4f, 0.7f, 1.0f, 1.0f};

      ImGui::InputText("Name", m_newBodyName, IM_ARRAYSIZE(m_newBodyName));
      ImGui::ColorEdit4("Color", &newColor.x);
      ImGui::DragFloat3("Position", &newPosition.x, 0.1f);
      ImGui::DragFloat3("Size", &newSize.x, 0.1f);
      ImGui::DragFloat("Mass", &newMass, 0.1f);

      ImGui::Separator();
      if (ImGui::Button("Create", ImVec2(120, 0))) {
        if (!m_system.bodyExists(m_newBodyName)) {
          Body body;
          body.position = newPosition;
          body.orientation = newOrientation;
          body.size = newSize;
          body.mass = newMass;
          body.color = newColor;
          m_system.addBody(m_newBodyName, body);
          m_selectedBody = m_newBodyName;
          ImGui::CloseCurrentPopup();
        }
      }
      ImGui::SameLine();
      if (ImGui::Button("Cancel", ImVec2(120, 0))) {
        ImGui::CloseCurrentPopup();
      }
      ImGui::EndPopup();
    }

    // Body List
    ImGui::BeginChild("BodyList",
                      ImVec2(0, -ImGui::GetFrameHeightWithSpacing()), true);
    for (const auto &[name, body]: m_system.bodies()) {
      bool isSelected = (m_selectedBody == name);
      ImVec4 textColor = ImVec4(body.color.r, body.color.g, body.color.b,
                                body.color.a);

      ImGui::PushStyleColor(ImGuiCol_Text, textColor);
      if (ImGui::Selectable(name.c_str(), isSelected)) {
        m_selectedBody = name;
        m_guizmo.setSelectedBody(name);
      }
      ImGui::PopStyleColor();

      if (isSelected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndChild();

    // Delete Selected
    if (!m_selectedBody.empty()) {
      if (ImGui::Button("- Delete Selected")) {
        m_system.removeBody(m_selectedBody);
        m_selectedBody.clear();
      }
    }
  }

  void renderBodyProperties() {
    auto &body = m_system.getBody(m_selectedBody);

    ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.5f, 1.0f), "Editing: %s",
                       m_selectedBody.c_str());
    ImGui::Separator();

    ImGui::ColorEdit4("Color", &body.color.x);
    ImGui::DragFloat3("Position", &body.position.x, 0.1f);
    ImGui::DragFloat3("Size", &body.size.x, 0.1f);
    ImGui::DragFloat("Mass", &body.mass, 0.1f, 0.1f, 1000.0f);
  }

  void resetSystem() {
    m_system.clear();
    m_selectedBody.clear();
    m_bodyCounter = 1;
  }
};

#endif // SYSTEMEDITOR_H
