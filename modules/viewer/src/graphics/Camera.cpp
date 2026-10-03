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
    /*
        View Matrix는 "카메라를 움직이는 행렬"이라기보다
        World 전체를 카메라 기준 좌표계로 옮기는 역변환으로 이해하면 쉽다.
        lookAt(position, target, up)이 camera basis를 만들어 이 변환을 계산한다.
    */
    return glm::lookAt(m_Position, m_Target, m_Up);
}

glm::mat4 Camera::GetProjectionMatrix() const
{
    /*
        Perspective projection은 멀리 있는 물체를 작게 보이게 한다.
        FOV는 degree로 저장하지만 GLM perspective는 radian을 요구하므로 변환한다.
    */
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
