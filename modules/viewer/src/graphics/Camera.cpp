#include "Camera.h"

#include <glm/gtc/matrix_transform.hpp>

#include <stdexcept>

Camera::Camera(
    const glm::vec3& position,
    const glm::vec3& target,
    float aspectRatio,
    float fov,
    float nearPlane,
    float farPlane)
    : m_Position(position),
      m_Target(target),
      m_AspectRatio(aspectRatio),
      m_Fov(fov),
      m_NearPlane(nearPlane),
      m_FarPlane(farPlane)
{
    if (aspectRatio <= 0.0F)
        throw std::runtime_error("Camera aspect ratio must be greater than 0");

    if (nearPlane <= 0.0F || farPlane <= nearPlane)
        throw std::runtime_error("Invalid camera clipping planes");
}

glm::mat4 Camera::GetViewMatrix() const
{
    return glm::lookAt(m_Position, m_Target, m_Up);
}

glm::mat4 Camera::GetProjectionMatrix() const
{
    return glm::perspective(
        glm::radians(m_Fov),
        m_AspectRatio,
        m_NearPlane,
        m_FarPlane);
}

void Camera::SetAspectRatio(float aspectRatio)
{
    if (aspectRatio <= 0.0F) return;
    m_AspectRatio = aspectRatio;
}

void Camera::SetPosition(const glm::vec3& position)
{
    m_Position = position;
}

void Camera::SetTarget(const glm::vec3& target)
{
    m_Target = target;
}

glm::vec3 Camera::GetPosition() const
{
    return m_Position;
}

glm::vec3 Camera::GetTarget() const
{
    return m_Target;
}
