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
/*
 * [추가 그래픽스 용어 설명]
 * - Pose: 카메라의 위치와 바라보는 방향을 합친 상태.
 * - View Matrix: World 좌표를 "카메라가 원점에 있다고 가정한 좌표"로 바꾸는 행렬.
 * - Projection Matrix: 3D Camera 좌표를 원근감이 있는 화면 좌표로 투영하는 행렬.
 * - Perspective: 멀수록 작게 보이도록 만드는 원근 투영 방식.
 * - FOV(Field of View): 카메라가 한 화면에 볼 수 있는 시야각. 여기서는 세로 각도 [deg].
 * - Aspect Ratio: 화면 width/height 비율. 단위 없음.
 * - Near/Far Clipping Plane: 이 거리보다 너무 가깝거나 먼 geometry를 그리지 않는 경계.
 * - Y-up: World의 위쪽 방향을 +Y축으로 사용하는 좌표계 규칙.
 *
 * 현재 HCR scene이 meter 기준이므로 position/target/near/far도 같은 scene에서 [m]로 해석한다.
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
    // 카메라가 실제로 놓인 World 위치. 현재 scene에서는 meter 단위.
    glm::vec3 m_Position;

    // 카메라가 바라보는 World 지점. m_Position과 같은 scene 단위.
    glm::vec3 m_Target;

    /// 현재 Viewer 런타임은 Y-up 좌표계를 사용한다.
    glm::vec3 m_Up{0.0F, 1.0F, 0.0F};

    // viewport width / height. 예: 1920/1080 ~= 1.777...
    float m_AspectRatio;

    // 세로 FOV [deg]. Projection 계산 직전에 radian으로 변환한다.
    float m_Fov;

    // 카메라보다 이 거리보다 가까운 geometry는 clipping된다. scene unit, 현재 HCR scene에서는 [m].
    float m_NearPlane;

    // 카메라보다 이 거리보다 먼 geometry는 clipping된다. scene unit, 현재 HCR scene에서는 [m].
    float m_FarPlane;
};
