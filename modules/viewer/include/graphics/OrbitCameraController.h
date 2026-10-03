#pragma once

class Camera;
class Window;

/**
 * @brief Robot/CAD Viewer에 적합한 target 중심 Orbit/Pan/Zoom 입력을 처리한다.
 *
 * @details
 * - Right Mouse Drag: target을 중심으로 회전(Orbit)
 * - Middle Mouse Drag: 화면 평면 기준 이동(Pan)
 * - Mouse Wheel: target과 camera 사이 거리 변경(Zoom)
 *
 * Orbit camera는 FPS camera와 달리 관심 대상(target)을 항상 유지하기 때문에
 * 로봇 관절과 작업공간을 여러 방향에서 검사하기 편하다.
 *
 * Camera는 최종 pose만 저장하고, Window는 raw input만 제공한다.
 * 이 Controller가 둘 사이의 입력 해석 책임을 담당한다.
 *
 * @todo [FUTURE] sensitivity와 min/max distance를 사용자 설정으로 이동한다.
 * @todo [FUTURE] 선택한 Entity를 새로운 orbit target으로 focus하는 기능을 추가할 수 있다.
 */
class OrbitCameraController
{
public:
    /** @brief 제어할 Camera와 입력을 읽을 Window를 연결한다. */
    OrbitCameraController(Camera& camera, Window& window);

    /** @brief PollEvents() 이후 매 frame 호출해 mouse 상태를 camera pose에 반영한다. */
    void OnUpdate();

private:
    /** @brief 현재/이전 cursor 위치를 비교해 drag delta를 계산한다. */
    void UpdateMouseState();

    /** @brief target 기준 yaw/pitch를 변경한다. */
    void Orbit(float deltaX, float deltaY);

    /** @brief camera와 target을 화면 right/up 방향으로 동시에 이동한다. */
    void Pan(float deltaX, float deltaY);

    /** @brief target과 camera 사이 거리를 지수형 비율로 변경한다. */
    void Zoom(float scrollOffset);

private:
    Camera& m_Camera;
    Window& m_Window;

    double m_LastMouseX = 0.0;
    double m_LastMouseY = 0.0;
    bool m_FirstMouseFrame = true;

    float m_OrbitSensitivity = 0.005F;
    float m_PanSensitivity = 0.0015F;
    float m_ZoomSensitivity = 0.15F;

    float m_MinDistance = 0.25F;
    float m_MaxDistance = 10.0F;
};
