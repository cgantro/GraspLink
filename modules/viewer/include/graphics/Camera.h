#pragma once

#include <glm/glm.hpp>


class Camera{
public:
    /*
        position -> 카메라의 위치
        target -> 바라보는 지점
        aspectRatio -> 화면의 가로 세로 비율 (가로/세로)
        fov -> 세로 시야각 (Field of View)
            -> 단위는 degree
        nearPlane / farPlane
            -> 카메라가 렌더링할 최소/최대 거리
    */
    Camera(
        const glm::vec3& position,
        const glm::vec3& target,
        float aspectRatio,
        float fov = 45.0f,
        float nearPlane = 0.1f,
        float farPlane = 100.f
    );

    /*
        View Matrix
        
        World 좌표를 Camera 기준 좌표로 변환한다.

        World 전체를 카메라의 반대방향으로 움직이게 한다.
    */
    glm::mat4 GetViewMatrix() const;
    /*
        Projectrion Matirx:
        카메라 공간의 3D 좌표에 원근감 적용
        
        멀리있는 물체 -> 작게
        가까이 있는 물체 -> 크게
    */
    glm::mat4 GetProjectionMatrix() const;

    /*
        윈도우 크기 변하면 aspect ratio 변경
    */
    void SetAspectRatio(float aspectRatio);
private:
    glm::vec3 m_Position;
    glm::vec3 m_Target;

    /*
        카메라의 위쪽 방향
        보통 +y 방향을 위쪽으로 사용
    */
    // 현재 뷰어가 사용하는 Y-up 런타임 좌표계에 맞춘 카메라 위쪽 방향이다.
    // Z-up을 사용하면 화면이 기울어지고, 바닥 평면의 법선(Y+)과도 일치하지 않는다.
    glm::vec3 m_Up{0.0F, 1.0F, 0.0F};

    float m_AspectRatio;
    float m_Fov;
    float m_NearPlane;
    float m_FarPlane;
};
