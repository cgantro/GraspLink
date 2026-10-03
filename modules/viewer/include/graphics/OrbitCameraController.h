#pragma once

class Camera;
class Window;

/*
    Robot/CAD Viewer용 Orbit Camera Controller.

    조작:
        Right Mouse Drag  -> Target 중심 Orbit
        Middle Mouse Drag -> 화면 기준 Pan
        Mouse Wheel        -> Zoom

    Camera는 최종 position/target과 View/Projection 계산만 담당하고,
    입력 해석은 이 Controller에서 담당한다.
*/
class OrbitCameraController
{
public:
    OrbitCameraController(Camera& camera, Window& window);

    // PollEvents() 이후 frame마다 한 번 호출한다.
    void OnUpdate();

private:
    void UpdateMouseState();
    void Orbit(float deltaX, float deltaY);
    void Pan(float deltaX, float deltaY);
    void Zoom(float scrollOffset);

private:
    Camera& m_Camera;
    Window& m_Window;

    double m_LastMouseX = 0.0;
    double m_LastMouseY = 0.0;
    bool m_FirstMouseFrame = true;

    // radian / pixel
    float m_OrbitSensitivity = 0.005F;

    // World 이동량은 camera-target 거리에 비례해서 추가 scaling한다.
    float m_PanSensitivity = 0.0015F;

    // Wheel 한 단계당 지수형 distance 변화율.
    float m_ZoomSensitivity = 0.15F;

    float m_MinDistance = 0.25F;
    float m_MaxDistance = 10.0F;
};
