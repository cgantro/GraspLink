#pragma once

#include "robotics/runtime/FixedControlLoop.h"

#include <flecs.h>
#include <memory>

class Window;
class Renderer;
class Camera;
class OrbitCameraController;
class SceneManager;
class AssetManager;
class Shader;
class PhysicsSystemModule;
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

/**
 * @brief Viewer Runtime 객체의 생성·연결·종료 순서 관리
 *
 * 흐름: Window → Camera / GLB → Entity / Controller → Transform / Physics → Entity / Flecs → Renderer
 * 역할: 세부 기능 구현 없이 객체 구성과 실행 순서만 담당
 */
class ViewerApp
{
public:
    ViewerApp();
    ~ViewerApp();

    int Run();

private:
    // 전체 초기화 순서
    bool Init();

    // Window·Renderer·Camera·ECS·Scene 준비
    bool InitViewer();

    // Robot·Floor GLB를 Scene Entity로 생성
    void InitScene(Entity& robotRoot, Entity& floorEntity);

    // Simulation Controller와 GLB Robot 연결
    bool InitRobot(const Entity& robotRoot);

    // PhysicsWorld와 Floor·Robot 설정 연결
    void InitPhysics(const Entity& robotRoot, Entity& floorEntity);

    // 입력·Simulation·ECS·Render 반복
    void MainLoop();

    // 참조 관계에 맞춰 Runtime 객체 정리
    void Shutdown();

    // GLFW Window·OpenGL Context 소유
    std::unique_ptr<Window> m_Window;

    // OpenGL Render 담당
    std::unique_ptr<Renderer> m_Renderer;

    // View·Projection 행렬 관리
    std::unique_ptr<Camera> m_Camera;

    // Mouse 입력 → Camera 조작
    std::unique_ptr<OrbitCameraController> m_CameraController;

    // Scene 생성·수명 관리
    std::unique_ptr<SceneManager> m_SceneManager;

    // GPU Mesh·Material·Texture 관리
    std::unique_ptr<AssetManager> m_AssetManager;

    // Robot·Debug Box 공용 Shader
    std::shared_ptr<Shader> m_RobotShader;

    // Flecs Entity·Component·System 소유
    flecs::world m_World;

    // Robot 제어 Backend
    std::unique_ptr<grasplink::robotics::IRobotController> m_RobotController;

    // RobotState 관절각 → GLB Joint 회전
    std::unique_ptr<grasplink::viewer::robotics::RobotTransformAdapter> m_RobotTransformAdapter;

    // Robot·Physics 고정 시간 업데이트
    grasplink::robotics::runtime::FixedControlLoop m_ControlLoop;

    // Jolt World 소유
    std::unique_ptr<grasplink::physics::PhysicsWorld> m_PhysicsWorld;

    // Flecs 물리 설정과 Jolt 연결·고정 Step 실행
    std::unique_ptr<PhysicsSystemModule> m_PhysicsSystemModule;
};
