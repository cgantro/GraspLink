#include "graphics/OrbitCameraController.h"

#include "graphics/Camera.h"
#include "graphics/Window.h"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>

namespace grasplink::graphics
{

OrbitCameraController::OrbitCameraController(Camera& camera, Window& window)
    : m_Camera(camera),
      m_Window(window)
{
}

void OrbitCameraController::OnUpdate()
{
    UpdateMouseState();

    // Window가 모은 세로 휠 이동량을 읽는다. 읽은 값은 0으로 비워 다음 입력과 섞이지 않게 한다.
    const float scrollOffset = static_cast<float>(m_Window.ConsumeScrollOffset());
    if (scrollOffset != 0.0F)
        Zoom(scrollOffset);
}

void OrbitCameraController::UpdateMouseState()
{
    double mouseX = 0.0;
    double mouseY = 0.0;
    m_Window.GetCursorPosition(mouseX, mouseY);

    // 처음 읽은 커서 위치는 다음 이동량을 재기 위한 기준점으로만 저장한다. 창을 연 위치 자체로 카메라가 튀지 않게 한다.
    if (m_FirstMouseFrame)
    {
        m_LastMouseX = mouseX;
        m_LastMouseY = mouseY;
        m_FirstMouseFrame = false;
        return;
    }

    // 마우스 좌표는 창의 논리 픽셀 [px]이다. 화면 위쪽으로 움직일수록 Y 좌표가 작아진다.
    const float deltaX = static_cast<float>(mouseX - m_LastMouseX);
    const float deltaY = static_cast<float>(mouseY - m_LastMouseY);

    m_LastMouseX = mouseX;
    m_LastMouseY = mouseY;

    // 오른쪽 버튼은 목표점을 중심으로 카메라를 돌리고, 가운데 버튼은 카메라와 목표점을 함께 옮긴다.
    // 두 버튼을 동시에 누르면 두 조작이 차례로 적용된다.
    if (m_Window.IsMouseButtonPressed(MouseButton::Right))
        Orbit(deltaX, deltaY);

    if (m_Window.IsMouseButtonPressed(MouseButton::Middle))
        Pan(deltaX, deltaY);
}

void OrbitCameraController::Orbit(float deltaX, float deltaY)
{
    const glm::vec3 target = m_Camera.GetTarget();
    glm::vec3 offset = m_Camera.GetPosition() - target;

    // 목표점에서 카메라로 향하는 선을 길이와 두 각도로 나눈다. 장면의 +Y가 위쪽인 좌표에서 좌우각과 위아래각을 구한다.
    // 선의 길이는 거리 [m], 좌우·위아래 각도는 [rad]다. 입력 좌표를 곱하는 감도도 [rad/px]다.
    const float radius = glm::length(offset);
    if (radius <= 0.0001F) return;

    float yaw = std::atan2(offset.z, offset.x);
    float pitch = std::asin(std::clamp(offset.y / radius, -1.0F, 1.0F));

    // 마우스를 오른쪽으로 끌 때 물체가 함께 움직이는 느낌이 들도록 좌우 회전각의 부호를 뒤집는다.
    yaw -= deltaX * m_OrbitSensitivity;
    pitch += deltaY * m_OrbitSensitivity;

    // Orbit의 위아래 회전은 수직 바로 아래로부터 약 1° 여유를 두고 제한한다. 정수리 방향을 넘으면 카메라 시선과 고정된 장면 위쪽(+Y)이 나란해져 화면의 오른쪽 방향을 계산하기 어려워진다.
    constexpr float kPitchLimit = 1.55334306F;
    pitch = std::clamp(pitch, -kPitchLimit, kPitchLimit);

    // 계산한 거리와 두 각도로 목표점에서 카메라까지의 장면 좌표 이동량을 만든다. 거리는 유지하고 방향만 바꾼다.
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

    // 카메라의 앞·오른쪽·위쪽 세 방향을 만든다. 앞쪽은 카메라에서 목표점으로 향하고 위쪽 기준은 장면의 +Y다.
    const glm::vec3 right = glm::normalize(glm::cross(forward, worldUp));
    const glm::vec3 up = glm::normalize(glm::cross(right, forward));

    const float distance = glm::length(target - position);

    // 카메라와 목표점 사이 거리 [m]에 비례해 옮긴다. 멀리서 움직일 때도 화면에서 비슷한 크기로 이동한다.
    const float scale = distance * m_PanSensitivity;

    // Pan은 카메라와 목표점을 같은 만큼 옮겨 바라보는 방향을 유지한다. 커서를 오른쪽으로 끌면 장면은 왼쪽으로, 위쪽으로 끌면 장면은 위로 이동하는 느낌이 난다.
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

    // Zoom은 Window가 모은 세로 스크롤 값으로 카메라와 목표점 사이 거리를 바꾼다. 거리에 비례하는 배율을 사용해 가까이 보거나 멀리 볼 때 조작감이 비슷하며 양수 입력은 목표점 쪽으로 다가간다.
    float newDistance = currentDistance * std::exp(-scrollOffset * m_ZoomSensitivity);
    newDistance = std::clamp(newDistance, m_MinDistance, m_MaxDistance);

    // 바라보는 방향은 유지한 채 카메라와 목표점 사이 거리만 허용 범위 안에 둔다. 목표점 위치는 바꾸지 않는다.
    const glm::vec3 direction = offset / currentDistance;
    m_Camera.SetPosition(target + direction * newDistance);
}

} // namespace grasplink::graphics
