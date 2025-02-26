#include "scenes/MainMenu.h"

#include <imgui.h>
#include <core/InputManager.h>
#include <core/SceneManager.h>

#include "scenes/GameLevel.h"
#include "utils/Logger.h"
#include "utils/OpenGLSetup.h"

bool MainMenu::load() {
  // Access resources via context
  // m_ctx.renderer->loadTexture("menu_bg", "assets/menu_bg.png");
  return true;
}

void MainMenu::update(float dt) {
  // Handle camera movement
  m_ctx.input->handleCameraMovement(*m_ctx.camera, dt);

  // Use input manager from context
  if (m_ctx.input->isKeyPressed(GLFW_KEY_ENTER)) {
    // Transition to GameLevel using scene manager from context
    m_ctx.scenes->pushScene(std::make_unique<GameLevel>(m_ctx));
    LOG_DEBUG("Transitioning to GameLevel");
  }
}

void MainMenu::render() {
    showCameraDebug();
}

void MainMenu::unload() {
}

void MainMenu::showCameraDebug() {
    Camera& camera = *m_ctx.camera;  // Alias for cleaner access

    // Use const references for vectors
    const glm::vec3& position = camera.getPosition();
    const glm::vec3& front = camera.getFront();
    const glm::vec3& left = camera.getLeft();
    const glm::vec3& up = camera.getUp();


    // Get float values (returned by value, safe)
    float yaw = camera.getYaw();
    float pitch = camera.getPitch();
    float fov = camera.getFov();
    float nearClip = camera.getNearClip();
    float farClip = camera.getFarClip();
    float moveSpeed = camera.getMovementSpeed();
    float mouseSensitivity = camera.getMouseSensitivity();

    ImGui::Begin("Camera Debug");

    // Use ImGui Table for better alignment
    if (ImGui::BeginTable("Camera Info", 2, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
        ImGui::TableSetupColumn("Property", ImGuiTableColumnFlags_WidthFixed, 120.0f);
        ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();

        #define DISPLAY_VECTOR(name, vec) \
            ImGui::TableNextRow(); \
            ImGui::TableNextColumn(); ImGui::Text(name); \
            ImGui::TableNextColumn(); ImGui::Text("(%.2f, %.2f, %.2f)", vec.x, vec.y, vec.z);

        DISPLAY_VECTOR("Position", position);
        DISPLAY_VECTOR("Front", front);
        DISPLAY_VECTOR("Left", left);
        DISPLAY_VECTOR("Up", up);

        #undef DISPLAY_VECTOR

        #define DISPLAY_FLOAT(name, value) \
            ImGui::TableNextRow(); \
            ImGui::TableNextColumn(); ImGui::Text(name); \
            ImGui::TableNextColumn(); ImGui::Text("%.2f", value);

        DISPLAY_FLOAT("Yaw", yaw);
        DISPLAY_FLOAT("Pitch", pitch);
        DISPLAY_FLOAT("FOV", fov);
        DISPLAY_FLOAT("Near Clip", nearClip);
        DISPLAY_FLOAT("Far Clip", farClip);
        DISPLAY_FLOAT("Move Speed", moveSpeed);
        DISPLAY_FLOAT("Mouse Sens.", mouseSensitivity);

        #undef DISPLAY_FLOAT

        ImGui::EndTable();
    }

    ImGui::Separator();

    // Editable parameters
    if (ImGui::SliderFloat("Yaw", &yaw, -180.0f, 180.0f)) {
        camera.setYaw(yaw);
    }
    if (ImGui::SliderFloat("Pitch", &pitch, -89.0f, 89.0f)) {
        camera.setPitch(pitch);
    }
    if (ImGui::SliderFloat("FOV", &fov, 10.0f, 120.0f)) {
        camera.setFov(fov);
    }
    if (ImGui::SliderFloat("Near Clip", &nearClip, 0.01f, 1.0f)) {
        camera.setNearClip(nearClip);
    }
    if (ImGui::SliderFloat("Far Clip", &farClip, 10.0f, 1000.0f)) {
        camera.setFarClip(farClip);
    }
    if (ImGui::SliderFloat("Move Speed", &moveSpeed, 1.0f, 20.0f)) {
        camera.setMovementSpeed(moveSpeed);
    }
    if (ImGui::SliderFloat("Mouse Sens.", &mouseSensitivity, 0.01f, 1.0f)) {
        camera.setMouseSensitivity(mouseSensitivity);
    }

    ImGui::End();
}
