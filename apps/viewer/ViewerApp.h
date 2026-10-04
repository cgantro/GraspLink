#pragma once

#include "PhysicsTypes.h"
#include "robotics/runtime/FixedControlLoop.h"

#include <flecs.h>
#include <memory>

class Window;
class Renderer;
class Camera;
class OrbitCameraController;
class SceneManager;
class AssetManager;
class PhysicsSyncSystem;
class Entity;

namespace grasplink::physics
{
class PhysicsWorld;
}

namespace grasplink::robotics
{
class IRobotController;
}

namespace grasplink::viewer::robotics
{
class RobotTransformAdapter;
}

/*
 * Viewer 실행에 필요한 객체를 생성하고 서로 연결한다.
 *
 * 주요 흐름:
 *
 * Window/Input -> Camera
 * GLB -> Flecs Entity
 * RobotController -> RobotTransformAdapter -> Flecs Transform
 * PhysicsWorld -> PhysicsSyncSystem -> Flecs Transform
 * Flecs -> Renderer -> OpenGL
 *
 * ViewerApp은 각 기능을 직접 구현하지 않고
 * Runtime 객체의 생성, 연결, 실행 순서를 관리한다.
 */
class ViewerApp
{
public:
    ViewerApp();
    ~ViewerApp();

    int Run();

private:
    // 전체 초기화 순서를 관리한다.
    bool Init();

    // Window, Renderer, Camera, ECS, Scene 기본 객체를 준비한다.
    bool InitViewer();

    // Robot과 바닥 GLB를 Scene에 생성하고 Robot Root를 반환한다.
    Entity InitScene();

    // Simulation Robot Controller와 GLB Robot을 연결한다.
    bool InitRobot(const Entity& robotRoot);

    // Jolt Physics World와 테스트 Body를 생성한다.
    void InitPhysics();

    // 입력, Simulation, ECS, Rendering을 반복 실행한다.
    void MainLoop();

    // 생성한 Runtime 객체를 안전한 순서로 정리한다.
    void Shutdown();

    // GLFW Window와 OpenGL Context.
    std::unique_ptr<Window> m_Window;

    // OpenGL Rendering 담당.
    std::unique_ptr<Renderer> m_Renderer;

    // View / Projection 상태.
    std::unique_ptr<Camera> m_Camera;

    // Mouse 입력을 Camera 이동으로 변환한다.
    std::unique_ptr<OrbitCameraController> m_CameraController;

    // Scene 생성과 수명 관리.
    std::unique_ptr<SceneManager> m_SceneManager;

    // Mesh / Material / Texture GPU Resource 관리.
    std::unique_ptr<AssetManager> m_AssetManager;

    // 모든 Flecs Entity / Component / System을 소유한다.
    flecs::world m_World;

    // 현재 사용하는 Robot Backend.
    std::unique_ptr<grasplink::robotics::IRobotController> m_RobotController;

    // RobotState 관절각을 GLB Joint Transform에 적용한다.
    std::unique_ptr<grasplink::viewer::robotics::RobotTransformAdapter> m_RobotTransformAdapter;

    // Robot과 Physics를 Rendering FPS와 관계없이 일정한 dt로 실행한다.
    grasplink::robotics::runtime::FixedControlLoop m_ControlLoop;

    // Jolt Physics World.
    std::unique_ptr<grasplink::physics::PhysicsWorld> m_PhysicsWorld;

    // Physics Body Transform을 Flecs Entity Transform에 반영한다.
    std::unique_ptr<PhysicsSyncSystem> m_PhysicsSyncSystem;

    // 현재 낙하 테스트용 Dynamic Box.
    grasplink::physics::PhysicsBodyHandle m_DebugBoxBody;
};