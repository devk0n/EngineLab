#include "core/Camera.h"


Camera::Camera() {
  setPosition(glm::vec3(10.0f, 8.0f, 6.0f));
  lookAt(glm::vec3(0.0f, 0.0f, 0.0f));
  updateCameraVectors();
}

glm::mat4 Camera::getViewMatrix() const {
  // Proper RHS lookAt with Z+ up
  return lookAtRH(m_position, m_position + m_front, m_up);
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

void Camera::lookAt(glm::vec3 target) {
  // Recompute forward dir
  glm::vec3 newFront = glm::normalize(target - m_position);

  m_yaw   = glm::degrees(std::atan2(newFront.y, newFront.x));
  m_pitch = glm::degrees(std::asin(newFront.z));

  updateCameraVectors();
}

void Camera::updateCameraVectors() {
  // Proper spherical coordinates for Z+ up RHS
  glm::vec3 front;
  front.x = cos(glm::radians(m_yaw)) * cos(glm::radians(m_pitch));
  front.y = sin(glm::radians(m_yaw)) * cos(glm::radians(m_pitch)); // Fixed Y component
  front.z = sin(glm::radians(m_pitch));                            // Fixed Z component
  m_front = normalize(front);

  // Right vector remains correct
  m_right = normalize(cross(m_front, glm::vec3(0.0f, 0.0f, 1.0f)));

  // Up vector calculation stays correct
  m_up = normalize(cross(m_right, m_front));
}

const glm::vec3 & Camera::getPosition() const {
  return m_position;
}

const glm::vec3 & Camera::getFront() const {
  return m_front;
}

const glm::vec3 & Camera::getUp() const {
  return m_up;
}

const glm::vec3 & Camera::getRight() const {
  return m_right;
}

float Camera::getYaw() const {
  return m_yaw;
}

float Camera::getPitch() const {
  return m_pitch;
}

float Camera::getFov() const {
  return m_fov;
}

float Camera::getAspectRatio() const {
  return m_aspectRatio;
}

float Camera::getNearClip() const {
  return m_nearClip;
}

float Camera::getFarClip() const {
  return m_farClip;
}

float Camera::getMovementSpeed() const {
  return m_movementSpeed;
}

float Camera::getMouseSensitivity() const {
  return m_mouseSensitivity;
}

void Camera::setPosition(glm::vec3 position) {
  m_position = position;
  updateCameraVectors();
}

void Camera::setFront(glm::vec3 front) {
  m_front = front;
  updateCameraVectors();
}

void Camera::setUp(glm::vec3 up) {
  m_up = up;
}

void Camera::setRight(glm::vec3 right) {
  m_right = right;
}

void Camera::setYaw(float yaw) {
  m_yaw = yaw;
}

void Camera::setPitch(float pitch) {
  m_pitch = pitch;
}

void Camera::setFov(float fov) {
  m_fov = fov;
}

void Camera::setAspectRatio(float aspectRatio) {
  m_aspectRatio = aspectRatio;
}

void Camera::setNearClip(float nearClip) {
  m_nearClip = nearClip;
}

void Camera::setFarClip(float farClip) {
  m_farClip = farClip;
}

void Camera::setMovementSpeed(float movementSpeed) {
  m_movementSpeed = movementSpeed;
}

void Camera::setMouseSensitivity(float mouseSensitivity) {
  m_mouseSensitivity = mouseSensitivity;
}
