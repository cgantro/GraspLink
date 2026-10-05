#pragma once

#include "robotics/runtime/FixedControlLoop.h"
#include "assets/GraphicsTypes.h"

#include <flecs.h>
#include <memory>

class Window;
class Renderer;
class Camera;
class OrbitCameraController;
class SceneManager;
class AssetManager;
class Shader;
class Entity;

namespace grasplink::physics
{
class PhysicsWorld;
}

namespace grasplink::robotics
{
class IRobotController;
namespace kinematics { class RobotKinematics; }
}

namespace grasplink::viewer::robotics
{
class RobotTransformAdapter;
}

namespace grasplink::gui
{
class GuiModule;
}

namespace grasplink::simulation
{
class PhysicsSystemModule;
class RobotPhysicsAdapter;
}

struct ViewerOptions
{
    bool physicsDemo = false;
    bool smokeTest = false;
};

// Runtime 객체를 조합하고 실행 순서와 수명을 관리한다. FK·물리·렌더 계산은 각 모듈에 맡긴다.
// Shutdown에서 빌린 참조를 먼저 정리하고, Entity/GPU 리소스를 World/OpenGL Context보다 먼저 해제한다.
class ViewerApp
{
public:
    explicit ViewerApp(ViewerOptions options = {});
    ~ViewerApp();

    int Run();

private:
    ViewerOptions m_Options;
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

    // 부분 초기화 실패 때도 호출된다. World observer와 OpenGL Context가 필요한 정리를 먼저 끝낸다.
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

    // Collider 추출용 CPU 데이터와 업로드된 GPU Mesh를 함께 공유한다. Context보다 먼저 해제한다.
    ModelResource m_RobotModel;

    // Robot·Debug Box 공용 Shader
    std::shared_ptr<Shader> m_RobotShader;

    // Flecs Entity·Component·System 소유
    flecs::world m_World;

    // Robot 제어 Backend
    std::unique_ptr<grasplink::robotics::IRobotController> m_RobotController;

    // Adapter는 Scene Entity handle을 빌린다. Scene 정리 전에 파괴한다.
    std::unique_ptr<grasplink::robotics::kinematics::RobotKinematics> m_RobotKinematics;
    std::unique_ptr<grasplink::viewer::robotics::RobotTransformAdapter> m_RobotTransformAdapter;
    std::unique_ptr<grasplink::simulation::RobotPhysicsAdapter> m_RobotPhysicsAdapter;

    // Robot·Physics 고정 시간 업데이트
    grasplink::robotics::runtime::FixedControlLoop m_ControlLoop;

    // Jolt World 소유
    std::unique_ptr<grasplink::physics::PhysicsWorld> m_PhysicsWorld;

    // Flecs 물리 설정과 Jolt 연결·고정 Step 실행
    std::unique_ptr<grasplink::simulation::PhysicsSystemModule> m_PhysicsSystemModule;

    // GUI lifecycle과 디버그 panel
    std::unique_ptr<grasplink::gui::GuiModule> m_GuiModule;
};
