#pragma once

namespace grasplink::graphics
{

class Camera;
class Window;

/**
 * @brief 마우스와 휠 입력으로 카메라 위치와 바라보는 점을 움직인다.
 * @details
 * Orbit은 목표점을 중심으로 카메라를 회전해 빙 둘러 보는 조작이다. Pan은 카메라와 목표점을 함께 옮겨 바라보는 방향을 유지하는 조작이며 Zoom은 목표점까지 거리를 바꾼다.
 * 카메라와 목표점은 장면 전체(World) 기준 위치 [m]로 저장한다. 오른쪽 드래그는 Orbit, 가운데 드래그는 Pan, 휠은 Zoom을 수행한다.
 * 마우스 변화량은 창의 논리 픽셀 [px]이며 회전 감도는 픽셀당 각도 [rad/px]다. Pan은 카메라 앞·오른쪽·위 방향으로 이동한다.
 * Camera와 Window는 이 Controller가 소유하지 않는다. 둘은 Controller보다 오래 살아야 한다. ViewerApp은 GUI가 마우스를 쓰는 동안 입력 갱신을 멈추고 종료할 때 Controller를 먼저 없앤다.
 */
class OrbitCameraController
{
public:

    /**
     * @brief 외부에서 수명을 관리하는 카메라와 창을 조작 대상으로 연결한다.
     * @param camera 위치와 바라볼 점을 읽고 갱신할 카메라. 이 객체보다 오래 살아야 한다.
     * @param window 커서·버튼·휠 입력을 읽을 창. 이 객체보다 오래 살아야 한다.
     */
    OrbitCameraController(Camera& camera, Window& window);

    /**
     * @brief 마우스 위치와 쌓인 휠 입력을 읽어 카메라 위치와 방향을 갱신한다.
     * @details
     * 처음 호출할 때는 커서 좌표만 기억해 창을 연 직후 카메라가 갑자기 움직이지 않게 한다. 이후 오른쪽 버튼은 목표점 주변 Orbit, 가운데 버튼은 Pan을 적용한다.
     * ViewerApp이 GUI에 마우스가 쓰이지 않을 때만 호출하므로 GUI 조작 중에는 카메라가 멈춘다.
     */
    void OnUpdate();

private:

    void UpdateMouseState();

    void Orbit(float deltaX, float deltaY);

    void Pan(float deltaX, float deltaY);

    void Zoom(float scrollOffset);

private:

    // 참조만 보관한다. 외부 소유자가 이 객체보다 먼저 파괴하면 안 된다.
    Camera& m_Camera;

    Window& m_Window;

    // 직전 커서 위치 [논리 px]. 버튼을 놓은 동안에도 갱신해 누른 순간 갑자기 크게 움직이지 않게 한다.
    double m_LastMouseX = 0.0;
    double m_LastMouseY = 0.0;

    bool m_FirstMouseFrame = true;

    // 커서 1 px 이동에 적용할 회전각 [rad/px]. 수평은 좌우, 수직은 위아래 회전으로 바뀐다.
    float m_OrbitSensitivity = 0.005F;

    // 커서 1 px 이동에 적용할 거리 비율. 카메라가 목표점에서 먼 만큼 실제 이동도 커진다.
    float m_PanSensitivity = 0.0015F;

    // 휠 한 단위에 적용할 거리 배율. 양수 입력은 목표점에 가까워지게 한다.
    float m_ZoomSensitivity = 0.15F;

    // 카메라와 목표점 사이 허용 거리 [m]. 목표점과 겹치거나 너무 멀리 이동하지 않게 한다.
    float m_MinDistance = 0.25F;
    float m_MaxDistance = 10.0F;
};

} // namespace grasplink::graphics
