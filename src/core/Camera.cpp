#include "core/Camera.h"

Camera::Camera() :
    m_position(0.0f, 0.0f, 0.0f), m_orientation(1.0f, 0.0f, 0.0f, 0.0f),
    m_movementSpeed(5.0f), m_mouseSensitivity(0.1f), m_fov(45.0f),
    m_aspectRatio(16.0f / 9.0f), m_nearClip(0.1f), m_farClip(100.0f) {
  updateVectors();
}

void Camera::processMouseMovement(float xOffset, float yOffset) {
  xOffset *= m_mouseSensitivity;
  yOffset *= m_mouseSensitivity;

  // Create rotation quaternions
  glm::quat yawRot =
      angleAxis(glm::radians(-xOffset), glm::vec3(0.0f, 0.0f, 1.0f));
  glm::quat pitchRot =
      angleAxis(glm::radians(-yOffset), -glm::vec3(0.0f, 1.0f, 0.0f));

  // Apply rotations (yaw first, then pitch)
  m_orientation = yawRot * m_orientation * pitchRot;

  // Normalize to prevent drift
  m_orientation = normalize(m_orientation);

  updateVectors();
}

void Camera::processKeyboardInput(CameraMovement direction, float deltaTime) {
  float velocity = m_movementSpeed * deltaTime;

  switch (direction) {
    case CameraMovement::FORWARD:
      m_position += m_front * velocity;
      break;
    case CameraMovement::BACKWARD:
      m_position -= m_front * velocity;
      break;
    case CameraMovement::LEFT:
      m_position += m_left * velocity;
      break;
    case CameraMovement::RIGHT:
      m_position -= m_left * velocity;
      break;
    case CameraMovement::UP:
      m_position += m_up * velocity;
      break;
    case CameraMovement::DOWN:
      m_position -= m_up * velocity;
      break;
  }
}

void Camera::updateVectors() {
  m_front = m_orientation * glm::vec3(1.0f, 0.0f, 0.0f);
  m_left = m_orientation * glm::vec3(0.0f, 1.0f, 0.0f);
  m_up = m_orientation * glm::vec3(0.0f, 0.0f, 1.0f);
}

glm::mat4 Camera::getViewMatrix() const {
  return lookAt(m_position, m_position + m_front, m_up);
}

glm::mat4 Camera::getProjectionMatrix() const {
  return glm::perspective(glm::radians(m_fov), m_aspectRatio, m_nearClip,
                          m_farClip);
}
