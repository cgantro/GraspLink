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
    // 화면 종횡비가 0이면 투영 폭을 계산할 수 없고, 잘라낼 깊이 구간이 아니면 원근 깊이를 정할 수 없다.
    if (aspectRatio <= 0.0F)
        throw std::runtime_error("Camera aspect ratio must be greater than 0");

    if (nearPlane <= 0.0F || farPlane <= nearPlane)
        throw std::runtime_error("Invalid camera clipping planes");
}

glm::mat4 Camera::GetViewMatrix() const
{
    // lookAt이 Target 방향을 -Z, Up에서 계산한 오른쪽을 +X로 삼아 camera pose의 역변환을 만든다.
    return glm::lookAt(m_Position, m_Target, m_Up);
}

glm::mat4 Camera::GetProjectionMatrix() const
{

    // OpenGL 원근 깊이는 비선형이다. near plane을 필요 이상으로 카메라에 붙이면 먼 거리 정밀도가 줄어든다.
    // FOV는 세로 각도이며 aspect ratio가 가로 각도를 결정한다. GLM은 FOV를 라디안으로 받는다.
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
