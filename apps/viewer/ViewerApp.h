#pragma once

#include <flecs.h>

#include <memory>

namespace PoseLink
{
class Window;
class Renderer;
class Camera;
class RenderSystem;
} // namespace PoseLink

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
private:
    /*
        Application 최상위에서 entity 수명과 component storage를 소유한다.

        Shutdown()에서는 world를 먼저 파괴한다. world 내부 Renderable의 shared_ptr가
        Mesh/Shader/Texture를 해제할 때 OpenGL context(Window)가 아직 살아 있어야 하기 때문이다.
    */
    std::unique_ptr<PoseLink::Window> m_Window;
    std::unique_ptr<PoseLink::Renderer> m_Renderer;
    std::unique_ptr<PoseLink::Camera> m_Camera;
    

    flecs::world m_World;

    float m_LastFrameTime = 0.0f;
};
