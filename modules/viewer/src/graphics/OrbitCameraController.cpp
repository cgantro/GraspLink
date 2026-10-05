#include "OrbitCameraController.h"

#include "Camera.h"
#include "Window.h"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>

OrbitCameraController::OrbitCameraController(Camera& camera, Window& window)
    : m_Camera(camera),
      m_Window(window)
{
}

void OrbitCameraController::OnUpdate()
{
    UpdateMouseState();

    // Window가 callback의 세로 scroll 누계를 한 번 반환하고 0으로 초기화한다.
    const float scrollOffset = static_cast<float>(m_Window.ConsumeScrollOffset());
    if (scrollOffset != 0.0F)
        Zoom(scrollOffset);
}

void OrbitCameraController::UpdateMouseState()
{
    double mouseX = 0.0;
    double mouseY = 0.0;
    m_Window.GetCursorPosition(mouseX, mouseY);

    // 첫 입력은 기준점만 저장한다. 창을 연 직후 커서의 절대 위치를 회전량으로 쓰지 않는다.
    if (m_FirstMouseFrame)
    {
        m_LastMouseX = mouseX;
        m_LastMouseY = mouseY;
        m_FirstMouseFrame = false;
        return;
    }

    // 좌표·단위: GLFW 논리 Window pixel [px]. 위쪽으로 움직이면 deltaY가 음수다.
    const float deltaX = static_cast<float>(mouseX - m_LastMouseX);
    const float deltaY = static_cast<float>(mouseY - m_LastMouseY);

    m_LastMouseX = mouseX;
    m_LastMouseY = mouseY;

    // 오른쪽 orbit과 가운데 pan은 각각 독립 입력으로 검사한다. 둘 다 누르면 두 동작이 순서대로 적용된다.
    if (m_Window.IsMouseButtonPressed(MouseButton::Right))
        Orbit(deltaX, deltaY);

    if (m_Window.IsMouseButtonPressed(MouseButton::Middle))
        Pan(deltaX, deltaY);
}

void OrbitCameraController::Orbit(float deltaX, float deltaY)
{
    const glm::vec3 target = m_Camera.GetTarget();
    glm::vec3 offset = m_Camera.GetPosition() - target;

    // offset 방향은 Target → Camera이며 World Y-up 구면 좌표로 풀어 각도를 더한다.
    // 반지름은 [m], yaw/pitch는 [rad]. atan2/asin 결과도 rad라 감도 [rad/px]와 단위가 맞는다.
    const float radius = glm::length(offset);
    if (radius <= 0.0001F) return;

    float yaw = std::atan2(offset.z, offset.x);
    float pitch = std::asin(std::clamp(offset.y / radius, -1.0F, 1.0F));

    // 화면 오른쪽 드래그는 물체를 따라 도는 느낌이 나도록 yaw를 반대로 바꾼다.
    yaw -= deltaX * m_OrbitSensitivity;
    pitch += deltaY * m_OrbitSensitivity;

    // 약 ±89°로 제한한다. 극점에서 시선과 Camera의 고정 World Y-up이 평행해지면
    // glm::lookAt이 right 축을 만들 때 외적 길이가 0에 가까워 화면 방향이 불안정해진다.
    constexpr float kPitchLimit = 1.55334306F;
    pitch = std::clamp(pitch, -kPitchLimit, kPitchLimit);

    // 구면 좌표를 World XYZ offset으로 복원한다. 반지름은 유지하고 방향만 회전한다.
    offset.x = radius * std::cos(pitch) * std::cos(yaw);
    offset.y = radius * std::sin(pitch);
    offset.z = radius * std::cos(pitch) * std::sin(yaw);

    m_Camera.SetPosition(target + offset);
}

void OrbitCameraController::Pan(float deltaX, float deltaY)
{
    const glm::vec3 position = m_Camera.GetPosition();
    const glm::vec3 target = m_Camera.GetTarget();

    const glm::vec3 forward = glm::normalize(target - position);
    constexpr glm::vec3 worldUp{0.0F, 1.0F, 0.0F};

    // Camera basis를 World Y-up 기준으로 만든다. forward는 Camera → target 방향이다.
    const glm::vec3 right = glm::normalize(glm::cross(forward, worldUp));
    const glm::vec3 up = glm::normalize(glm::cross(right, forward));

    const float distance = glm::length(target - position);

    // 이동 단위 [m]: 거리 비례 배율로 멀리서도 화면상 pan 속도를 비슷하게 유지한다.
    const float scale = distance * m_PanSensitivity;

    // 같은 translation을 Camera와 target에 더해 시선은 유지한다. 커서 오른쪽은 장면을 왼쪽으로,
    // 위쪽(deltaY 음수)은 장면을 위로 옮기는 방향이다.
    const glm::vec3 translation =
        -right * deltaX * scale +
         up * deltaY * scale;

    m_Camera.SetPosition(position + translation);
    m_Camera.SetTarget(target + translation);
}

void OrbitCameraController::Zoom(float scrollOffset)
{
    const glm::vec3 target = m_Camera.GetTarget();
    glm::vec3 offset = m_Camera.GetPosition() - target;

    const float currentDistance = glm::length(offset);
    if (currentDistance <= 0.0001F) return;

    // scroll offset은 Window callback의 세로 누계 단위다. 지수 배율은 거리에 비례해
    // 이동하므로 가까운 곳과 먼 곳에서 비슷한 조작감을 주며 양수는 반지름을 줄인다.
    float newDistance = currentDistance * std::exp(-scrollOffset * m_ZoomSensitivity);
    newDistance = std::clamp(newDistance, m_MinDistance, m_MaxDistance);

    // 방향은 유지하고 반지름만 제한된 거리로 바꾼다. target 자체는 움직이지 않는다.
    const glm::vec3 direction = offset / currentDistance;
    m_Camera.SetPosition(target + direction * newDistance);
}
