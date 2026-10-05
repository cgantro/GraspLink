#pragma once

#include <flecs.h>

#include <memory>

class Camera;
class Window;
namespace grasplink::robotics { class IGripperController; }

namespace grasplink::gui
{

/**
 * @brief ECS Collider 설정을 화면 위 선으로 표시하고 ImGui 입력 상태를 제공한다.
 * @details
 * flecs::world, Window, Camera는 ViewerApp에서 빌린 참조이며 이 모듈이 소유하지 않는다. 따라서 query와
 * ImGui/OpenGL backend를 정리할 때 세 객체가 살아 있어야 한다. 선은 ECS Colliders 설정을 투영한 근사
 * 표시다. Jolt Body/Shape를 조회하지 않으며 실제 충돌 형상과 다를 수 있다.
 */
class GuiModule
{
public:
    GuiModule(flecs::world& world, Window& window, Camera& camera);
    ~GuiModule();

    GuiModule(const GuiModule&) = delete;
    GuiModule& operator=(const GuiModule&) = delete;

    /**
     * @brief ImGui 패널과 설정 Collider 선을 현재 프레임에 그린다.
     * @details Collider 선과 투영 결과는 약 100 ms 간격으로 다시 계산해 캐시한다. Screen 좌표 캐시이므로
     * 그 사이 Camera가 움직여도 다음 갱신 전까지 선이 이전 위치에 남을 수 있고, 깊이 검사 없이 전경에 그린다.
     * 선택적 Gripper 패널은 전달된 Controller에만 명령을 요청하고 상태를 표시한다. 포인터는 이 호출 동안만 빌리며 저장하지 않는다.
     * 인자를 생략하면 기존 Collider 패널만 그린다.
     * @param gripper 이번 프레임에 사용할 선택적 Gripper Controller. nullptr이면 Gripper 패널을 표시하지 않는다.
     */
    void Draw(::grasplink::robotics::IGripperController* gripper = nullptr);

    /** @brief ImGui가 마우스를 잡고 있어 Camera 입력을 막아야 하는지 알려준다. */
    bool WantsMouse() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_Impl;
};

}
