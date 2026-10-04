#pragma once

#include <flecs.h>
#include <memory>

class Window;
class Renderer;
class Camera;
class OrbitCameraController;
class SceneManager;
class AssetManager;

namespace grasplink::robotics
{
class IRobotController;
}

namespace grasplink::viewer::robotics
{
class RobotTransformAdapter;
}

/**
 * @brief Viewer 실행 객체를 조립하고 전체 생명주기를 관리하는 Composition Root.
 */
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
    std::unique_ptr<OrbitCameraController> m_CameraController;

    std::unique_ptr<SceneManager> m_SceneManager;
    std::unique_ptr<AssetManager> m_AssetManager;

    std::unique_ptr<grasplink::robotics::IRobotController> m_RobotController;
    std::unique_ptr<grasplink::viewer::robotics::RobotTransformAdapter> m_RobotTransformAdapter;

    flecs::world m_World;
};
