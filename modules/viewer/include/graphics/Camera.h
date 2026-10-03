#pragma once

#include <glm/glm.hpp>

class Camera{
public:
    /*
        position -> 카메라의 위치
        target -> 바라보는 지점
        aspectRatio -> 화면의 가로 세로 비율 (가로/세로)
        fov -> 세로 시야각 (Field of View), 단위는 degree
        nearPlane / farPlane -> 렌더링 최소/최대 거리
    */
    Camera(
        const glm::vec3& position,
        const glm::vec3& target,
        float aspectRatio,
        float fov = 45.0F,
        float nearPlane = 0.1F,
        float farPlane = 100.0F
    );

    // World 좌표를 Camera 좌표계로 변환하는 View Matrix.
    glm::mat4 GetViewMatrix() const;

    // Perspective Projection Matrix.
    glm::mat4 GetProjectionMatrix() const;

    // Window resize 시 projection의 종횡비를 갱신한다.
    void SetAspectRatio(float aspectRatio);

    /*
        OrbitCameraController가 Camera의 pose를 변경할 때 사용한다.
        Camera 자체는 입력을 해석하지 않고 최종 position/target만 보관한다.
    */
    void SetPosition(const glm::vec3& position);
    void SetTarget(const glm::vec3& target);

    glm::vec3 GetPosition() const;
    glm::vec3 GetTarget() const;

private:
    glm::vec3 m_Position;
    glm::vec3 m_Target;

    // 현재 Viewer는 Y-up 좌표계를 사용한다.
    glm::vec3 m_Up{0.0F, 1.0F, 0.0F};

    float m_AspectRatio;
    float m_Fov;
    float m_NearPlane;
    float m_FarPlane;
};
