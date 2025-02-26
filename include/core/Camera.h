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

  // Get matrices
  [[nodiscard]] glm::mat4 getViewMatrix() const;
  [[nodiscard]] glm::mat4 getProjectionMatrix() const;

  // Camera controls
  void processKeyboardInput(CameraMovement direction, float deltaTime);
  void processMouseMovement(float xOffset, float yOffset, bool constrainPitch = true);
  void processScroll(float yOffset);

  glm::vec3 getPosition() const;

  float *getMovementSpeed();
  float *getMouseSensitivity();

private:
  glm::vec3 m_position;
  glm::vec3 m_front;
  glm::vec3 m_up;
  glm::vec3 m_right;

  // Euler angles
  float m_yaw;
  float m_pitch;

  // Camera settings
  float m_movementSpeed = 5.0f;
  float m_mouseSensitivity = 0.1f;

  // Projection parameters
  float m_fov = 45.0f;
  float m_aspectRatio = 16.0f / 9.0f;
  float m_nearClip = 0.1f;
  float m_farClip = 250.0f;

  void updateCameraVectors();
};


#endif // CAMERA_H
