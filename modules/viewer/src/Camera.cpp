#include "Camera.h"

#include <glm/gtc/matrix_transform.hpp>

#include <stdexcept>

namespace PoseLink
{
Camera::Camera(
    const glm::vec3& position,
    const glm::vec3& target,
    float aspectRatio,
    float fov,
    float nearPlane,
    float farPlane
) : m_Position(position),
    m_Target(target),
    m_AspectRatio(aspectRatio),
    m_Fov(fov),
    m_NearPlane(nearPlane),
    m_FarPlane(farPlane){
    
    if (aspectRatio <= 0.0f)
        throw std::runtime_error(
            "Camera aspect ratio must be greater than 0"
        );

    if (nearPlane <= 0.0f || farPlane <= nearPlane)
        throw std::runtime_error(
            "Invalid camera clipping planes"
        );
}

glm::mat4 Camera::GetViewMatrix() const{
    /*
        glm::lookAt -> 카메라의 위치와 방향을 가지고 View Matrix 생성
    */
    return glm::lookAt(m_Position,m_Target,m_Up);
}

glm::mat4 Camera::GetProjectionMatrix() const{
    /*
        Perspective Projection 생성
        glm::perspective()
            fov -> radian 단위 요구
            따라서 degree로 저장한 m_Fov를 glm::radian으로 변환
    */

    return glm::perspective(
        glm::radians(m_Fov),
        m_AspectRatio,
        m_NearPlane,
        m_FarPlane
    );
}

void Camera::SetAspectRatio(float aspectRatio){
    if(aspectRatio <= 0.0f) return;
    m_AspectRatio = aspectRatio;
}
} // namespace PoseLink
