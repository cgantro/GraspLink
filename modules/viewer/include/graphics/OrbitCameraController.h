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
/*
 * [추가 그래픽스 용어 설명]
 * - Orbit: target을 중심으로 카메라 위치가 구면을 따라 도는 조작.
 * - Pan: 카메라 방향은 유지한 채 camera와 target을 화면 좌/우/위/아래로 함께 이동하는 조작.
 * - Zoom: 여기서는 FOV를 바꾸는 방식이 아니라 camera-target 거리 자체를 바꾸는 방식.
 * - Yaw: 위쪽 축을 기준으로 좌우 회전하는 각도.
 * - Pitch: 좌우 축을 기준으로 위/아래로 기울이는 각도.
 * - Cursor delta: 이번 frame과 이전 frame의 mouse 좌표 차이. 입력값은 pixel 단위다.
 * - Sensitivity: 입력 delta를 실제 회전/이동량으로 바꾸는 배율.
 *
 * m_LastMouseX/Y는 window cursor 좌표 [pixel].
 * m_MinDistance/m_MaxDistance는 camera-target 거리이며 현재 HCR scene에서는 [m]로 해석한다.
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
    // 이 Controller가 pose를 갱신할 Camera. 소유하지 않는 reference다.
    Camera& m_Camera;

    // mouse/scroll raw input을 읽을 Window. 소유하지 않는 reference다.
    Window& m_Window;

    // 이전 frame의 cursor 위치 [pixel].
    double m_LastMouseX = 0.0;
    double m_LastMouseY = 0.0;

    // 첫 frame에는 이전 좌표가 없으므로 큰 delta가 튀지 않도록 초기화 여부를 표시한다.
    bool m_FirstMouseFrame = true;

    // Mouse pixel delta를 orbit angle 변화량으로 바꾸는 배율.
    float m_OrbitSensitivity = 0.005F;

    // Mouse pixel delta를 pan 이동량으로 바꾸는 배율.
    float m_PanSensitivity = 0.0015F;

    // Scroll 입력을 camera-target 거리 변화 비율로 바꾸는 배율.
    float m_ZoomSensitivity = 0.15F;

    // camera-target 최소/최대 거리. 현재 scene에서는 meter 기준으로 해석한다.
    float m_MinDistance = 0.25F;
    float m_MaxDistance = 10.0F;
};
