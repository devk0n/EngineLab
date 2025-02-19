#include "core/Camera.h"


Camera::Camera()
    : m_position(0.0f, 0.0f, -3.0f),
      m_front(0.0f, 1.0f, 0.0f),   // Looking forward
      m_up(0.0f, 0.0f, 1.0f),      // Z+ is up
      m_right(1.0f, 0.0f, 0.0f),   // X+ is right
      m_yaw(90.0f),                // Default facing along Y+
      m_pitch(0.0f),               // No tilt initially
      m_fov(45.0f),
      m_aspectRatio(16.0f / 9.0f), // This should be set dynamically
      m_nearClip(0.1f),
      m_farClip(100.0f) {
  updateCameraVectors();
}

glm::mat4 Camera::getViewMatrix() const {
  return glm::lookAt(m_position, m_position + m_front, m_up);
}

glm::mat4 Camera::getProjectionMatrix() const {
  return glm::perspective(glm::radians(m_fov), m_aspectRatio, m_nearClip, m_farClip);
}

void Camera::processKeyboardInput(CameraMovement direction, float deltaTime) {
  float velocity = m_movementSpeed * deltaTime;

  if (direction == CameraMovement::FORWARD)
    m_position += m_front * velocity;
  if (direction == CameraMovement::BACKWARD)
    m_position -= m_front * velocity;
  if (direction == CameraMovement::LEFT)
    m_position -= m_right * velocity;
  if (direction == CameraMovement::RIGHT)
    m_position += m_right * velocity;
  if (direction == CameraMovement::UP)
    m_position += m_up * velocity;
  if (direction == CameraMovement::DOWN)
    m_position -= m_up * velocity;
}

void Camera::processMouseMovement(float xOffset, float yOffset, bool constrainPitch) {
  xOffset *= m_mouseSensitivity;
  yOffset *= m_mouseSensitivity;

  m_yaw += xOffset;
  m_pitch += yOffset;

  // Constrain pitch to avoid flipping
  if (constrainPitch) {
    if (m_pitch > 89.0f) m_pitch = 89.0f;
    if (m_pitch < -89.0f) m_pitch = -89.0f;
  }

  updateCameraVectors();
}

void Camera::processScroll(float yOffset) {
  m_movementSpeed += yOffset;
}

glm::vec3 Camera::getPosition() {
  return m_position;
}

void Camera::updateCameraVectors() {
  // Convert Euler angles (yaw, pitch) to a direction vector
  glm::vec3 front;
  front.x = std::cos(glm::radians(m_yaw)) * std::cos(glm::radians(m_pitch));
  front.y = std::sin(glm::radians(m_yaw)) * std::cos(glm::radians(m_pitch));
  front.z = std::sin(glm::radians(m_pitch));
  m_front = glm::normalize(front);

  // Recalculate the right and up vectors
  m_right = glm::normalize(glm::cross(m_front, glm::vec3(0.0f, 0.0f, 1.0f))); // Always orthogonal to world up
  m_up    = glm::normalize(glm::cross(m_right, m_front));
}
