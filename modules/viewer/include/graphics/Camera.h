#pragma once

#include <glm/glm.hpp>

/**
 * @brief View/Projection 계산에 필요한 카메라 pose와 projection 파라미터를 보관한다.
 *
 * @details
 * Camera는 입력 장치에 대해 알지 않는다. OrbitCameraController가 mouse 입력을 해석한 뒤
 * position/target을 이 객체에 전달하고, Camera는 그 결과로 View/Projection matrix를 만든다.
 * 이 분리는 "입력 처리"와 "카메라 수학"의 책임을 분리하기 위한 것이다.
 *
 * @todo [FUTURE] Y-up 외 좌표계가 필요해지면 up vector를 생성자/설정값으로 분리한다.
 */
class Camera
{
public:
    /**
     * @brief Perspective camera를 생성한다.
     * @param position World 공간 카메라 위치.
     * @param target 카메라가 바라보는 World 공간 지점.
     * @param aspectRatio viewport width / height.
     * @param fov 세로 시야각(degree).
     * @param nearPlane near clipping plane 거리.
     * @param farPlane far clipping plane 거리.
     */
    Camera(
        const glm::vec3& position,
        const glm::vec3& target,
        float aspectRatio,
        float fov = 45.0F,
        float nearPlane = 0.1F,
        float farPlane = 100.0F);

    /** @brief World 좌표를 Camera 좌표계로 변환하는 View Matrix를 반환한다. */
    glm::mat4 GetViewMatrix() const;

    /** @brief Perspective Projection Matrix를 반환한다. */
    glm::mat4 GetProjectionMatrix() const;

    /** @brief Window resize 후 projection 종횡비를 갱신한다. */
    void SetAspectRatio(float aspectRatio);

    /** @brief Camera World position을 변경한다. */
    void SetPosition(const glm::vec3& position);

    /** @brief Camera가 바라볼 target point를 변경한다. */
    void SetTarget(const glm::vec3& target);

    /** @brief 현재 World position을 반환한다. */
    glm::vec3 GetPosition() const;

    /** @brief 현재 target point를 반환한다. */
    glm::vec3 GetTarget() const;

private:
    glm::vec3 m_Position;
    glm::vec3 m_Target;

    /// 현재 Viewer 런타임은 Y-up 좌표계를 사용한다.
    glm::vec3 m_Up{0.0F, 1.0F, 0.0F};

    float m_AspectRatio;
    float m_Fov;
    float m_NearPlane;
    float m_FarPlane;
};
