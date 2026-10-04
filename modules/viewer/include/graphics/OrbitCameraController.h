#pragma once

class Camera;
class Window;

/**
 * @brief Robot/CAD Viewer에 적합한 target 중심 Orbit/Pan/Zoom 입력을 처리한다.
 *
 * @details
 * 입력 매핑:
 * - Right Mouse Drag: target 중심 회전(Orbit)
 * - Middle Mouse Drag: 화면 평면 기준 평행 이동(Pan)
 * - Mouse Wheel: target-camera 거리 변경(Zoom)
 *
 * Camera는 최종 pose만 저장하고 Window는 raw input만 제공한다. 이 Controller가 두 객체 사이에서
 * cursor pixel delta/scroll 값을 camera의 world-space pose 변화로 변환한다.
 *
 * 단위/변환:
 * - Cursor deltaX/deltaY: Window pixel 단위
 * - Orbit sensitivity: radian/pixel
 * - Pan sensitivity: distance 대비 pixel 이동 비율. 실제 이동량은 `cameraDistance * sensitivity * pixelDelta`
 * - Zoom sensitivity: scroll step에 대한 지수 계수. `newDistance = distance * exp(-scroll * sensitivity)`
 * - min/max distance: Camera와 target 사이 scene distance. 현재 HCR scene에서는 [m]
 *
 * @todo [FUTURE] sensitivity와 min/max distance를 사용자 설정으로 이동한다.
 * @todo [FUTURE] 선택한 Entity를 새로운 orbit target으로 focus하는 기능을 추가할 수 있다.
 */
class OrbitCameraController
{
public:
    /**
     * @brief 제어할 Camera와 raw mouse input을 읽을 Window를 연결한다.
     * @param camera pose를 변경할 non-owning Camera reference.
     * @param window cursor/button/scroll 상태를 읽을 non-owning Window reference.
     * @note Camera와 Window는 이 Controller보다 오래 살아야 한다.
     */
    OrbitCameraController(Camera& camera, Window& window);

    /**
     * @brief PollEvents() 이후 매 frame 호출해 mouse drag/scroll을 camera pose에 반영한다.
     * @note Scroll 누적값은 이 함수에서 ConsumeScrollOffset()으로 한 번 소비한다.
     */
    void OnUpdate();

private:
    /** @brief 현재/이전 cursor 위치[pixel]를 비교해 drag delta를 계산하고 버튼별 동작으로 전달한다. */
    void UpdateMouseState();

    /**
     * @brief target-camera offset을 spherical coordinate로 바꿔 yaw/pitch를 회전한다.
     * @param deltaX 수평 cursor 이동량 [pixel].
     * @param deltaY 수직 cursor 이동량 [pixel].
     */
    void Orbit(float deltaX, float deltaY);

    /**
     * @brief Camera와 target을 화면 right/up 방향으로 같은 거리만큼 이동한다.
     * @param deltaX 수평 cursor 이동량 [pixel].
     * @param deltaY 수직 cursor 이동량 [pixel].
     */
    void Pan(float deltaX, float deltaY);

    /**
     * @brief target-camera 거리를 지수 비율로 변경한다.
     * @param scrollOffset Window가 누적한 vertical wheel scroll step.
     */
    void Zoom(float scrollOffset);

private:
    /** @brief 제어 대상 Camera의 non-owning reference. */
    Camera& m_Camera;

    /** @brief 입력 공급 Window의 non-owning reference. */
    Window& m_Window;

    /** @brief 이전 frame cursor X [pixel]. */
    double m_LastMouseX = 0.0;

    /** @brief 이전 frame cursor Y [pixel]. */
    double m_LastMouseY = 0.0;

    /** @brief 첫 frame에 큰 가짜 cursor delta가 생기는 것을 막는 초기화 flag. */
    bool m_FirstMouseFrame = true;

    /** @brief Orbit 회전량 계수 [rad/pixel]. */
    float m_OrbitSensitivity = 0.005F;

    /** @brief Pan translation 비율 [1/pixel]. 실제 world 이동량은 camera distance와 함께 계산된다. */
    float m_PanSensitivity = 0.0015F;

    /** @brief Scroll step에 대한 지수 zoom 계수. */
    float m_ZoomSensitivity = 0.15F;

    /** @brief target-camera 최소 거리. scene unit, 현재 HCR scene에서는 [m]. */
    float m_MinDistance = 0.25F;

    /** @brief target-camera 최대 거리. scene unit, 현재 HCR scene에서는 [m]. */
    float m_MaxDistance = 10.0F;
};
