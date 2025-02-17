#ifndef CAMERA_H
#define CAMERA_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

class Camera {
public:
    Camera() {
        updateVectors();
    }

    void moveForward(float deltaTime) {
        m_position += m_speed * deltaTime * m_front;
    }

    void moveBackward(float deltaTime) {
        m_position -= m_speed * deltaTime * m_front;
    }

    void moveLeft(float deltaTime) {
        m_position -= m_speed * deltaTime * m_right;
    }

    void moveRight(float deltaTime) {
        m_position += m_speed * deltaTime * m_right;
    }

    void moveUp(float deltaTime) {
        m_position += m_speed * deltaTime * m_up;
    }

    void moveDown(float deltaTime) {
        m_position -= m_speed * deltaTime * m_up;
    }

    glm::mat4 getViewMatrix() const {
        return glm::lookAt(m_position, m_position + m_front, m_up);
    }

    void setPosition(const glm::vec3& position) {
        m_position = position;
    }

    glm::vec3 getPosition() const {
        return m_position;
    }

    void processMouseMovement(float xoffset, float yoffset, bool constrainPitch = true) {
        m_yaw += xoffset;
        m_pitch += yoffset;

        if (constrainPitch) {
            if (m_pitch > 89.0f) m_pitch = 89.0f;
            if (m_pitch < -89.0f) m_pitch = -89.0f;
        }

        updateVectors();
    }

    void setSpeed(float speed) {
        m_speed = speed;
    }

    float getSpeed() const {
        return m_speed;
    }

private:
    glm::vec3 m_position = glm::vec3(0.0f, 0.0f, 3.0f);
    glm::vec3 m_front = glm::vec3(0.0f, 0.0f, -1.0f);
    glm::vec3 m_up = glm::vec3(0.0f, 1.0f, 0.0f);
    glm::vec3 m_right = glm::vec3(1.0f, 0.0f, 0.0f);
    float m_speed = 5.0f;
    float m_yaw = -90.0f;
    float m_pitch = 0.0f;

    void updateVectors() {
        glm::vec3 front;
        front.x = cos(glm::radians(m_yaw)) * cos(glm::radians(m_pitch));
        front.y = sin(glm::radians(m_pitch));
        front.z = sin(glm::radians(m_yaw)) * cos(glm::radians(m_pitch));
        m_front = glm::normalize(front);

        m_right = glm::normalize(glm::cross(m_front, glm::vec3(0.0f, 1.0f, 0.0f)));
        m_up = glm::normalize(glm::cross(m_right, m_front));
    }
};

#endif // CAMERA_H