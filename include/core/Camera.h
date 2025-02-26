#ifndef CAMERA_H
#define CAMERA_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

enum class CameraMovement {
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
  UP,
  DOWN
};

class Camera {
public:
  Camera();

  // Camera controls
  void processKeyboardInput(CameraMovement direction, float deltaTime);
  void processMouseMovement(float xOffset, float yOffset, bool constrainPitch = true);
  void processScroll(float yOffset);

  void lookAt(glm::vec3 target);

  // Get matrices
  [[nodiscard]] glm::mat4 getViewMatrix() const;
  [[nodiscard]] glm::mat4 getProjectionMatrix() const;

  // Return const references for vectors to avoid copying
  const glm::vec3& getPosition() const;
  const glm::vec3& getFront() const;
  const glm::vec3& getUp() const;
  const glm::vec3& getLeft() const;

  // For fundamental types, return by value to avoid issues
  float getYaw() const;
  float getPitch() const;
  float getFov() const;
  float getAspectRatio() const;
  float getNearClip() const;
  float getFarClip() const;
  float getMovementSpeed() const;
  float getMouseSensitivity() const;

  // Setters
  void setPosition(glm::vec3 position);
  void setYaw(float yaw);
  void setPitch(float pitch);
  void setFov(float fov);
  void setAspectRatio(float aspectRatio);
  void setNearClip(float nearClip);
  void setFarClip(float farClip);
  void setMovementSpeed(float movementSpeed);
  void setMouseSensitivity(float mouseSensitivity);

private:
  glm::vec3 m_position = glm::vec3(0.0f, 0.0f, 0.0f);
  glm::vec3 m_front = glm::vec3(1.0f, 0.0f, 0.0f);
  glm::vec3 m_left  = glm::vec3(0.0f, 1.0f, 0.0f);
  glm::vec3 m_up    = glm::vec3(0.0f, 0.0f, 1.0f);

  // Euler angles
  float m_roll = 0.0f;
  float m_pitch = 0.0f;
  float m_yaw = 0.0f;

  // Camera settings
  float m_movementSpeed = 10.0f;
  float m_mouseSensitivity = 0.1f;

  // Projection parameters
  float m_fov = 45.0f;
  float m_aspectRatio = 16.0f / 9.0f;
  float m_nearClip = 0.1f;
  float m_farClip = 250.0f;

  void updateCameraVectors();
};


#endif // CAMERA_H
