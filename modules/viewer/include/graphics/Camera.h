#pragma once

#include <glm/glm.hpp>

/**
 * @brief World 공간의 시점과 원근 투영 설정을 보관한다.
 * @details
 * 역할: 입력 처리와 분리된 값 객체. OrbitCameraController가 pose를 바꾸고 Renderer가 행렬을 읽는다.
 * 좌표: position/target은 World 공간의 점, +Y가 위쪽. m_Position은 카메라의 World 위치(cameraWorld의
 * 이동 성분)이며 별도의 cameraWorld 행렬을 저장하지 않는다. GLM lookAt이 position과 target에서
 * view 행렬을 계산하므로 pose 변경은 다음 행렬 조회부터 반영된다.
 * 흐름: 정점은 shader에서 `u_Projection * u_View * worldPosition` 순서로 변환된다. GLM은 열 벡터를
 * 오른쪽에서 곱하므로 먼저 World → camera(view), 이어 camera → clip(projection)로 간다.
 * 단위: World 거리와 clipping plane은 [m], 세로 FOV는 [deg]이며 GLM 호출 직전에 [rad]로 변환한다.
 * 투영 후 clip 좌표를 w로 나눈 NDC에서 x/y는 [-1, 1], OpenGL 깊이 z도 [-1, 1] 범위에 대응한다.
 */
class Camera
{
public:

    /**
     * @brief 바라볼 방향과 원근 투영 범위를 설정한다.
     * @param position World 공간 카메라 위치 [m]
     * @param target World 공간에서 바라볼 점 [m]
     * @param aspectRatio framebuffer 너비/높이 비율. 0보다 커야 함
     * @param fov 세로 시야각 [deg]
     * @param nearPlane 카메라 앞쪽 clipping plane까지 거리 [m]. 0보다 커야 함
     * @param farPlane 카메라 앞쪽 clipping plane까지 거리 [m]. nearPlane보다 커야 함
     * @throws std::runtime_error aspectRatio 또는 clipping plane 조건이 잘못된 경우
     * @details
     * glm::lookAt은 target - position 방향을 카메라의 전방(-Z)으로 두고 m_Up을 참고해 오른쪽(+X)과
     * 위쪽(+Y) 축을 정한다. perspective는 세로 FOV와 가로/세로 비율에서 화면 경계를 만들고,
     * nearPlane~farPlane 밖의 점은 clip 단계에서 화면에 나타나지 않는다. 종횡비와 plane의 대소 조건은
     * 검사하지만 FOV 범위와 입력의 유한성은 검사하지 않으므로 호출자는 유효한 값을 전달해야 한다.
     */
    Camera(
        const glm::vec3& position,
        const glm::vec3& target,
        float aspectRatio,
        float fov = 45.0F,
        float nearPlane = 0.1F,
        float farPlane = 100.0F);

    /**
     * @brief World 점을 카메라 기준 좌표로 옮기는 view 행렬을 반환한다.
     * @return GLM 열 벡터 기준 World → camera 행렬
     * @details
     * 카메라 위치를 원점으로 옮기고 바라보는 방향을 -Z에 맞추는 pose의 역변환이다.
     * m_Position과 m_Target은 World 공간 점이며, m_Up은 방향을 정하는 World 기준 위쪽 벡터다.
     */
    glm::mat4 GetViewMatrix() const;

    /**
     * @brief 카메라 좌표를 원근 clip 좌표로 보내는 투영 행렬을 반환한다.
     * @return Camera → clip 행렬. perspective 분할 뒤 NDC 깊이는 OpenGL 기준 [-1, 1]
     * @details
     * 세로 FOV와 aspect ratio가 화면의 가로·세로 시야를 정하고 near/far plane이 보이는 깊이 구간을
     * 자른다. 원근 분할로 먼 물체는 작아진다. 깊이는 거리와 선형으로 배분되지 않아 nearPlane을
     * 지나치게 작게 두면 표현 가능한 깊이 값이 가까운 구간에 집중돼 먼 표면끼리 구분하기 어려워진다.
     */
    glm::mat4 GetProjectionMatrix() const;

    /** @brief framebuffer 크기에 맞게 화면 가로/세로 비율을 갱신한다. @param aspectRatio 너비/높이, 0보다 큰 값만 반영 */
    void SetAspectRatio(float aspectRatio);

    /** @brief World 공간 카메라 위치 [m]를 갱신한다. @param position 새 World 위치 */
    void SetPosition(const glm::vec3& position);

    /** @brief World 공간에서 바라볼 점 [m]을 갱신한다. @param target 새 World 목표점 */
    void SetTarget(const glm::vec3& target);

    /** @return World 공간 카메라 위치 [m] */
    glm::vec3 GetPosition() const;

    /** @return World 공간에서 바라보는 목표점 [m] */
    glm::vec3 GetTarget() const;

private:
    glm::vec3 m_Position;

    glm::vec3 m_Target;

    glm::vec3 m_Up{0.0F, 1.0F, 0.0F};

    float m_AspectRatio;

    float m_Fov;

    float m_NearPlane;

    float m_FarPlane;
};
