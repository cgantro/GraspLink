#pragma once

#include <glm/glm.hpp>

/**
 * @brief View/Projection 계산에 필요한 camera pose와 perspective projection 파라미터를 보관한다.
 *
 * @details
 * Camera는 입력 장치에 대해 알지 않는다. OrbitCameraController가 mouse 입력을 해석해
 * position/target을 갱신하고 Camera는 그 상태에서 View/Projection matrix만 계산한다.
 *
 * 공간 단위는 Scene/asset과 동일한 world unit을 사용한다. 현재 HCR controller-ready asset은 meter 기준이므로
 * camera position/target/near/far도 같은 scene에서 meter [m]로 해석한다.
 * FOV만 degree [deg]로 저장하고 GetProjectionMatrix()에서 GLM이 요구하는 radian으로 변환한다.
 *
 * @todo [FUTURE] Y-up 외 좌표계가 필요해지면 up vector를 생성자/설정값으로 분리한다.
 */
class Camera
{
public:
    /**
     * @brief Perspective camera를 생성한다.
     * @param position World 공간 카메라 위치. 단위는 scene unit, 현재 HCR scene에서는 [m].
     * @param target 카메라가 바라보는 World 공간 지점. position과 동일 단위.
     * @param aspectRatio viewport width / height. 무차원, 0보다 커야 한다.
     * @param fov 세로 시야각 [deg]. 내부 projection 계산 시 radian으로 변환한다.
     * @param nearPlane near clipping plane까지의 거리. scene unit, 현재 HCR scene에서는 [m].
     * @param farPlane far clipping plane까지의 거리. scene unit, nearPlane보다 커야 한다.
     * @throws std::runtime_error aspectRatio<=0 또는 clipping plane 관계가 잘못된 경우.
     */
    Camera(
        const glm::vec3& position,
        const glm::vec3& target,
        float aspectRatio,
        float fov = 45.0F,
        float nearPlane = 0.1F,
        float farPlane = 100.0F);

    /**
     * @brief World 좌표를 Camera/View 좌표계로 변환하는 View Matrix를 계산한다.
     * @return `glm::lookAt(position, target, up)`으로 만든 4x4 matrix.
     */
    glm::mat4 GetViewMatrix() const;

    /**
     * @brief 현재 FOV/aspect/near/far로 Perspective Projection Matrix를 계산한다.
     * @return GLM/OpenGL convention의 4x4 projection matrix.
     */
    glm::mat4 GetProjectionMatrix() const;

    /**
     * @brief Window resize 후 projection 종횡비를 갱신한다.
     * @param aspectRatio framebuffer width/height. 0 이하 값은 무시한다.
     */
    void SetAspectRatio(float aspectRatio);

    /** @brief Camera World position을 변경한다. 단위는 scene world unit. */
    void SetPosition(const glm::vec3& position);

    /** @brief Camera가 바라볼 World target point를 변경한다. 단위는 scene world unit. */
    void SetTarget(const glm::vec3& target);

    /** @return 현재 Camera World position. 단위는 scene world unit. */
    glm::vec3 GetPosition() const;

    /** @return 현재 Camera target point. 단위는 scene world unit. */
    glm::vec3 GetTarget() const;

private:
    /** @brief Camera eye position in world space. */
    glm::vec3 m_Position;

    /** @brief Camera가 바라보는 world-space target point. */
    glm::vec3 m_Target;

    /** @brief Viewer의 world up 방향. 현재 좌표계는 Y-up. */
    glm::vec3 m_Up{0.0F, 1.0F, 0.0F};

    /** @brief Viewport width/height 종횡비. */
    float m_AspectRatio;

    /** @brief Vertical field of view [deg]. */
    float m_Fov;

    /** @brief Near clipping distance in scene units. */
    float m_NearPlane;

    /** @brief Far clipping distance in scene units. */
    float m_FarPlane;
};
