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

    // Scroll은 callback에서 누적되므로 frame당 한 번 소비한다.
    const float scrollOffset = static_cast<float>(m_Window.ConsumeScrollOffset());
    if (scrollOffset != 0.0F)
        Zoom(scrollOffset);
}

void OrbitCameraController::UpdateMouseState()
{
    double mouseX = 0.0;
    double mouseY = 0.0;
    m_Window.GetCursorPosition(mouseX, mouseY);

    /*
        첫 frame에는 이전 cursor 위치가 없으므로 delta를 만들지 않는다.
        그렇지 않으면 프로그램 시작 시 큰 delta가 생겨 Camera가 튈 수 있다.
    */
    if (m_FirstMouseFrame)
    {
        m_LastMouseX = mouseX;
        m_LastMouseY = mouseY;
        m_FirstMouseFrame = false;
        return;
    }

    const float deltaX = static_cast<float>(mouseX - m_LastMouseX);
    const float deltaY = static_cast<float>(mouseY - m_LastMouseY);

    // 버튼을 누르지 않는 frame에도 갱신해서 다음 drag 시작 시 jump가 발생하지 않게 한다.
    m_LastMouseX = mouseX;
    m_LastMouseY = mouseY;

    if (m_Window.IsMouseButtonPressed(MouseButton::Right))
        Orbit(deltaX, deltaY);

    if (m_Window.IsMouseButtonPressed(MouseButton::Middle))
        Pan(deltaX, deltaY);
}

void OrbitCameraController::Orbit(float deltaX, float deltaY)
{
    const glm::vec3 target = m_Camera.GetTarget();
    glm::vec3 offset = m_Camera.GetPosition() - target;

    const float radius = glm::length(offset);
    if (radius <= 0.0001F) return;

    /*
        Target -> Camera offset을 spherical coordinate로 변환한다.
        Y-up이므로 yaw는 Y축 회전, pitch는 위/아래 각도다.
    */
    float yaw = std::atan2(offset.z, offset.x);
    float pitch = std::asin(std::clamp(offset.y / radius, -1.0F, 1.0F));

    yaw -= deltaX * m_OrbitSensitivity;
    pitch += deltaY * m_OrbitSensitivity;

    /*
        pitch가 ±90도에 도달하면 view direction과 up vector가 평행해져
        lookAt이 불안정해질 수 있으므로 약 ±89도로 제한한다.
    */
    constexpr float kPitchLimit = 1.55334306F; // glm::radians(89.0F)
    pitch = std::clamp(pitch, -kPitchLimit, kPitchLimit);

    // spherical -> Cartesian
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

    // 화면의 오른쪽/위쪽 basis를 만든다.
    const glm::vec3 right = glm::normalize(glm::cross(forward, worldUp));
    const glm::vec3 up = glm::normalize(glm::cross(right, forward));

    const float distance = glm::length(target - position);
    const float scale = distance * m_PanSensitivity;

    /*
        Camera와 Target을 같은 벡터만큼 이동하면 시선 방향은 유지되고
        화면 전체가 평행 이동하는 Pan이 된다.
    */
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

    /*
        distance -= constant 방식보다 현재 거리에 비례하는 지수형 zoom이
        가까운 거리와 먼 거리 모두에서 조작감이 일정하다.
    */
    float newDistance = currentDistance * std::exp(-scrollOffset * m_ZoomSensitivity);
    newDistance = std::clamp(newDistance, m_MinDistance, m_MaxDistance);

    const glm::vec3 direction = offset / currentDistance;
    m_Camera.SetPosition(target + direction * newDistance);
}
