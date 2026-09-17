#pragma once

#include <flecs.h>
#include <memory>


class Window;
class Renderer;
class Camera;

// 창, 카메라, 렌더 시스템만 조립하는 최소 실행 진입점이다.
class ViewerApp
{
public:
    ViewerApp();
    ~ViewerApp();

    int Run();

private:
    bool Init();
    void MainLoop();
    void Shutdown();

    std::unique_ptr<Window> window_;
    std::unique_ptr<Renderer> renderer_;
    std::unique_ptr<Camera> camera_;
    flecs::world m_World;
    flecs::entity debugEntity_{flecs::entity::null()};
};
