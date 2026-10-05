#pragma once

class Camera;
class Window;

/**
 * @brief Window의 마우스 입력으로 Camera의 위치와 바라볼 점을 조정한다.
 * @details
 * Camera pose는 World 좌표 [m]로 저장한다. 오른쪽 버튼 드래그는 target 주위 회전,
 * 가운데 버튼 드래그는 시선 기준 평행 이동, 세로 scroll은 target 방향 dolly로 해석한다.
 * 커서 변화량은 논리 Window pixel [px]이고 orbit 감도는 [rad/px]이다. 이동 방향은
 * 현재 Camera의 전방·오른쪽·위쪽 축으로 바꾸며, pan 거리는 Camera-target 거리와 함께 커진다.
 * 입력을 읽을 때 참조하는 Camera와 Window는 호출자가 소유한다. ViewerApp은 입력을 갱신할 때
 * GUI가 마우스를 사용 중인지 먼저 확인하고, 종료 시 이 Controller를 두 객체보다 먼저 해제한다.
 */
class OrbitCameraController
{
public:

    /**
     * @brief 외부 소유 Camera와 Window를 조작 대상으로 연결한다.
     * @param camera 위치와 target을 읽고 갱신할 Camera. Controller보다 오래 살아야 한다.
     * @param window 커서, 버튼, scroll 입력을 제공하는 Window. Controller보다 오래 살아야 한다.
     */
    OrbitCameraController(Camera& camera, Window& window);

    /**
     * @brief 현재 커서와 누적 scroll 입력을 읽어 Camera pose를 갱신한다.
     * @details
     * 첫 호출은 커서 위치만 저장해 초기화 시 발생하는 큰 delta를 막는다. 이후 오른쪽 버튼은
     * orbit, 가운데 버튼은 pan을 적용한다. ViewerApp은 ImGui가 마우스를 점유하지 않을 때만
     * 이 함수를 부르므로 GUI 조작 중 Camera 입력은 멈추며, 다시 호출되면 누적 scroll을 소비한다.
     */
    void OnUpdate();

private:

    void UpdateMouseState();

    void Orbit(float deltaX, float deltaY);

    void Pan(float deltaX, float deltaY);

    void Zoom(float scrollOffset);

private:

    // 수명: 참조만 보유한다. 호출자가 두 객체를 먼저 파괴하지 않아야 한다.
    Camera& m_Camera;

    Window& m_Window;

    // 직전 논리 Window cursor 좌표 [px]. 버튼을 누르지 않는 동안에도 갱신해 drag 시작 점프를 막는다.
    double m_LastMouseX = 0.0;
    double m_LastMouseY = 0.0;

    bool m_FirstMouseFrame = true;

    // 회전량 [rad/px]. 화면 pixel 변화가 target 주위 yaw/pitch 변화로 변환된다.
    float m_OrbitSensitivity = 0.005F;

    // 거리 비율 [1/px]. 실제 평행 이동량은 Camera-target 거리에도 비례한다.
    float m_PanSensitivity = 0.0015F;

    // scroll 단위당 지수 배율. 양의 scroll은 target 쪽으로 가까워진다.
    float m_ZoomSensitivity = 0.15F;

    // Camera-target 거리 제한 [m]. 원점 퇴화와 지나치게 먼 dolly를 막는다.
    float m_MinDistance = 0.25F;
    float m_MaxDistance = 10.0F;
};
