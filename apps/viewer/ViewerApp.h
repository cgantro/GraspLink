#pragma once

#include <flecs.h>
#include <memory>


class Window;
class Renderer;
class Camera;
class SceneManager;
class AssetManager;
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

    std::unique_ptr<Window> m_Window;
    std::unique_ptr<Renderer> m_Renderer;
    std::unique_ptr<Camera> m_Camera;
    std::unique_ptr<SceneManager> m_SceneManager;
    std::unique_ptr<AssetManager> m_AssetManager;

    flecs::world m_World;
};
