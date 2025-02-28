#include "core/Camera.h"

Camera::Camera() :
    m_position(0.0f, 0.0f, 0.0f), m_orientation(1.0f, 0.0f, 0.0f, 0.0f),
    m_movementSpeed(5.0f), m_mouseSensitivity(0.1f), m_fov(45.0f),
    m_aspectRatio(16.0f / 9.0f), m_nearClip(0.1f), m_farClip(100.0f) {
  updateVectors();
}

void Camera::processMouseMovement(float xOffset, float yOffset,
                                  bool constrainPitch) {
  xOffset *= m_mouseSensitivity;
  yOffset *= m_mouseSensitivity;

  // Create rotation quaternions
  glm::quat yawRot =
      angleAxis(glm::radians(-xOffset), glm::vec3(0.0f, 0.0f, 1.0f));
  glm::quat pitchRot =
      angleAxis(glm::radians(-yOffset), glm::vec3(0.0f, -1.0f, 0.0f));

  // Apply rotations (yaw first, then pitch)
  m_orientation = yawRot * m_orientation * pitchRot;

  // Normalize to prevent drift
  m_orientation = normalize(m_orientation);

  updateVectors();
}

void Camera::processKeyboardInput(int direction, float deltaTime) {
  float velocity = m_movementSpeed * deltaTime;
  glm::vec3 moveDir(0.0f);

  if (direction == 0)
    moveDir += m_front; // Forward (W)
  if (direction == 1)
    moveDir -= m_front; // Backward (S)
  if (direction == 2)
    moveDir += m_left; // Left (A)
  if (direction == 3)
    moveDir -= m_left; // Right (D)
  if (direction == 4)
    moveDir += m_up; // Up (Space)
  if (direction == 5)
    moveDir -= m_up; // Down (Ctrl)

  if (glm::length(moveDir) > 0.0f) {
    m_position += glm::normalize(moveDir) * velocity;
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
