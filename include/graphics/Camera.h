#ifndef CAMERA_H
#define CAMERA_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

enum class CameraMovement { FORWARD, BACKWARD, LEFT, RIGHT, UP, DOWN };

class Camera {
public:
  Camera();

  // Camera controls
  void processMouseMovement(float xOffset, float yOffset);
  void processKeyboardInput(CameraMovement direction, float deltaTime);
  void processScroll(float yOffset);
  void lookAt(const glm::vec3 &target);

  // Getters
  glm::mat4 getViewMatrix() const;
  glm::mat4 getProjectionMatrix() const;
  glm::vec3 getPosition() const { return m_position; }
  glm::quat getOrientation() const { return m_orientation; }
  glm::vec3 getFront() const { return m_front; }
  glm::vec3 getLeft() const { return m_left; }
  glm::vec3 getUp() const { return m_up; }

  float getFov() const { return m_fov; }
  float getAspectRatio() const { return m_aspectRatio; }
  float getNearClip() const { return m_nearClip; }
  float getFarClip() const { return m_farClip; }
  float getMovementSpeed() const { return m_movementSpeed; }

  // Setters
  void setPosition(glm::vec3 position) { m_position = position; }
  void setOrientation(glm::quat orientation) { m_orientation = orientation; }
  void setFront(glm::vec3 front) { m_front = front; }
  void setLeft(glm::vec3 left) { m_left = left; }
  void setUp(glm::vec3 up) { m_up = up; }
  void setFov(float fov) { m_fov = fov; }
  void setAspectRatio(float aspectRatio) { m_aspectRatio = aspectRatio; }

  static inline glm::vec3 s_forward = {1.0f, 0.0f, 0.0f};
  static inline glm::vec3 s_left = {0.0f, 1.0f, 0.0f};
  static inline glm::vec3 s_up = {0.0f, 0.0f, 1.0f};

private:
  void updateVectors();

  // Camera state
  glm::vec3 m_position;
  glm::quat m_orientation;

  // Camera vectors
  glm::vec3 m_front = {1.0f, 0.0f, 0.0f};
  glm::vec3 m_left = {0.0f, 1.0f, 0.0f};
  glm::vec3 m_up = {0.0f, 0.0f, 1.0f};

  // Camera options
  float m_movementSpeed;
  float m_mouseSensitivity;

  float m_fov;
  float m_aspectRatio;
  float m_nearClip;
  float m_farClip;
};

#endif // CAMERA_H
