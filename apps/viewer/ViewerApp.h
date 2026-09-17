#pragma once

#include <flecs.h>
#include <memory>

namespace PoseLink
{
class Window;
class Renderer;
class Camera;
}

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

    std::unique_ptr<PoseLink::Window> window_;
    std::unique_ptr<PoseLink::Renderer> renderer_;
    std::unique_ptr<PoseLink::Camera> camera_;
    flecs::world world_;
    flecs::entity debugEntity_{flecs::entity::null()};
};
