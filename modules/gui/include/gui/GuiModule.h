#pragma once

#include <flecs.h>

#include <memory>

class Camera;
class Window;

namespace grasplink::gui
{

// ECS에 설정된 Collider를 화면에 투영한다. 실제 Jolt Body/Shape 상태를 조회하지 않는다.
// World·Window·Camera는 ViewerApp에서 빌린다. GUI를 World와 OpenGL Context보다 먼저 파괴한다.
class GuiModule
{
public:
    GuiModule(flecs::world& world, Window& window, Camera& camera);
    ~GuiModule();

    GuiModule(const GuiModule&) = delete;
    GuiModule& operator=(const GuiModule&) = delete;

    // 화면 선과 Camera 투영은 약 100 ms 간격으로 갱신하며, 깊이 검사 없이 그린다.
    void Draw();
    bool WantsMouse() const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_Impl;
};

}
