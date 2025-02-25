#include "core/Camera.h"


Camera::Camera() :
    m_position(0.0f, 0.0f, 3.0f),  // X=right, Y=forward, Z=up
    m_front(1.0f, 0.0f, 0.0f),    // Looking along negative X
    m_up(0.0f, 0.0f, 1.0f),        // World up is Z+
    m_right(0.0f, 1.0f, 0.0f),     // Right vector starts as Y+
    m_yaw(-90.0f),                 // Aligns with initial front vector
    m_pitch(0.0f) {
  updateCameraVectors(); // Force proper initialization
}

glm::mat4 Camera::getViewMatrix() const {
  // Proper RHS lookAt with Z+ up
  return glm::lookAtRH(m_position, m_position + m_front, m_up);
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
    m_pitch = glm::clamp(m_pitch, -89.0f, 89.0f);
  }

  updateCameraVectors();
}

void Camera::processScroll(float yOffset) {
  m_movementSpeed += yOffset;
}

glm::vec3 Camera::getPosition() const {
  return m_position;
}

float *Camera::getMovementSpeed() {
  return &m_movementSpeed;
}

float *Camera::getMouseSensitivity() {
  return &m_mouseSensitivity;
}

void Camera::updateCameraVectors() {
  // Proper spherical coordinates for Z+ up RHS
  glm::vec3 front;
  front.x = cos(glm::radians(m_yaw)) * cos(glm::radians(m_pitch));
  front.y = sin(glm::radians(m_yaw)) * cos(glm::radians(m_pitch)); // Fixed Y component
  front.z = sin(glm::radians(m_pitch));                            // Fixed Z component
  m_front = glm::normalize(front);

  // Right vector remains correct
  m_right = glm::normalize(glm::cross(m_front, glm::vec3(0.0f, 0.0f, 1.0f)));

  // Up vector calculation stays correct
  m_up = glm::normalize(glm::cross(m_right, m_front));
}
