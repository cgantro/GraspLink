#include "ViewerApp.h"

#include "Camera.h"
#include "Entity.h"
#include "OrbitCameraController.h"
#include "RenderContext.h"
#include "Renderer.h"
#include "RenderSystemModule.h"
#include "Shader.h"
#include "TransformSystemModule.h"
#include "Window.h"

#include "assets/AssetManager.h"
#include "assets/GltfLoader.h"
#include "assets/PrefabFactory.h"

#include "components/PhysicsComponents.h"

#include "PhysicsWorld.h"
#include "systems/PhysicsSystemModule.h"

#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/core/IRobotController.h"
#include "robotics/models/hanwha/Hcr12a.h"

#include "scene/Scene.h"
#include "scene/SceneManager.h"
#include "scene/EntityFactory.h"

#include "viewer/robotics/RobotTransformAdapter.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>

namespace
{

// 창 크기 (px)
constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 720;

// Robot·Physics 고정 주기: 250Hz (4ms)
constexpr double kControlFrequencyHz = 250.0;
constexpr double kControlFixedDeltaSeconds = 1.0 / kControlFrequencyHz;

// 긴 멈춤 뒤 한 프레임의 Simulation 폭주 방지 (최대 100ms)
constexpr double kMaxFrameDeltaSeconds = 0.1;

const char* kWindowTitle = "GraspLink Viewer";

// Camera 시작 위치·목표 위치 (World 기준)
const glm::vec3 kCameraPosition{2.0F, 1.35F, 1.15F};
const glm::vec3 kCameraTarget{0.05F, 0.50F, 0.40F};

// 자주 쓰는 타입 별칭
using SimRobotController = grasplink::robotics::backends::simulation::SimRobotController;
using RobotTransformAdapter = grasplink::viewer::robotics::RobotTransformAdapter;
using PhysicsWorld = grasplink::physics::PhysicsWorld;

} // namespace


ViewerApp::ViewerApp()
    : m_ControlLoop(kControlFixedDeltaSeconds, kMaxFrameDeltaSeconds)
{
}


ViewerApp::~ViewerApp()
{
    Shutdown();
}


int ViewerApp::Run()
{
    // 초기화 실패 시 실행 중단
    if (!Init())
        return -1;

    MainLoop();
    return 0;
}


bool ViewerApp::Init()
{
    // 초기화 순서: Viewer → Scene·모델 → Robot Controller → Physics
    if (!InitViewer())
        return false;

    Entity robotRoot;
    Entity floorEntity;
    InitScene(robotRoot, floorEntity);

    if (!InitRobot(robotRoot))
        return false;

    InitPhysics(robotRoot, floorEntity);

    return true;
}


bool ViewerApp::InitViewer()
{
    // Window 생성. OpenGL Context도 Window가 소유
    m_Window = std::make_unique<Window>(
        Window::Properties{kWindowWidth, kWindowHeight, kWindowTitle, true});

    int framebufferWidth = 0;
    int framebufferHeight = 0;
    m_Window->GetFramebufferSize(framebufferWidth, framebufferHeight);

    // DPI 배율이 반영된 실제 Framebuffer 크기로 Renderer 초기화
    m_Renderer = std::make_unique<Renderer>();
    m_Renderer->Init(framebufferWidth, framebufferHeight);

    // 화면 비율: 가로 / 세로
    const float aspectRatio = static_cast<float>(kWindowWidth) / static_cast<float>(kWindowHeight);
    m_Camera = std::make_unique<Camera>(kCameraPosition, kCameraTarget, aspectRatio);

    // Mouse 입력 → Camera 회전·이동·확대
    m_CameraController = std::make_unique<OrbitCameraController>(*m_Camera, *m_Window);

    // RenderSystem이 사용할 Renderer·Camera 등록
    m_World.set<RenderContext>({m_Renderer.get(), m_Camera.get()});

    // ECS System: Transform 계산 → 렌더 대상 전달
    m_World.import<TransformSystemModule>();
    m_World.import<RenderSystemModule>();

    // 기본 Scene 생성
    m_SceneManager = std::make_unique<SceneManager>(m_World);
    m_SceneManager->LoadScene<Scene>();

    // Scene 초기 상태 반영
    m_SceneManager->OnUpdate(0.0F);

    // GPU Mesh·Texture 관리 객체
    m_AssetManager = std::make_unique<AssetManager>();

    return true;
}


void ViewerApp::InitScene(Entity& robotRoot, Entity& floorEntity)
{
    Scene* scene = m_SceneManager->GetActiveScene();

    // Robot·Floor 전용 Shader
    auto robotShader = Shader::Create("shaders/Robot.glsl");
    auto gridShader = Shader::Create("shaders/Grid.glsl");

    // GLB 읽기 → GPU Mesh·Material 업로드
    ModelResource robotModel = GltfLoader::LoadGLB("HCR12A_2F-85.glb");
    m_AssetManager->UploadModel(robotModel);

    // GLB Node 계층 → Flecs Entity 계층
    // robotRoot: 이후 J1~J6 검색 기준
    robotRoot = PrefabFactory::CreateModel(
        *scene,
        robotModel,
        *m_AssetManager,
        robotShader);

    // Floor Mesh 생성. Static Collider는 InitPhysics에서 같은 Entity에 연결
    ModelResource planeModel = GltfLoader::LoadGLB("plane.glb");
    m_AssetManager->UploadModel(planeModel);

    floorEntity = PrefabFactory::CreateModel(
        *scene,
        planeModel,
        *m_AssetManager,
        gridShader,
        true);

}


bool ViewerApp::InitRobot(const Entity& robotRoot)
{
    // HCR-12A 관절 수·축·제한·최대 속도
    const auto& robotSpec = grasplink::robotics::models::hanwha::kHcr12a;

    // 실제 Hardware 대신 Simulation Controller 사용
    auto simController = std::make_unique<SimRobotController>(robotSpec);

    const auto connectResult = simController->Connect();

    if (!connectResult)
    {
        std::cerr << connectResult.message << '\n';
        return false;
    }

    // 이후 제어는 구현체 대신 IRobotController 인터페이스 사용
    m_RobotController = std::move(simController);

    // 데이터 흐름: RobotState 관절각 → Adapter → GLB Joint Local 회전
    m_RobotTransformAdapter = std::make_unique<RobotTransformAdapter>(robotRoot, robotSpec);

    // Debug 전용: J1 30도 이동 확인
#ifndef NDEBUG
    grasplink::robotics::JointMoveCommand debugMove;

    debugMove.targetPositionRadians.assign(robotSpec.jointCount, 0.0);
    debugMove.targetPositionRadians[0] = glm::radians(30.0);
    debugMove.velocityScale = 1.0;
    debugMove.accelerationScale = 1.0;

    const auto moveResult = m_RobotController->MoveJoint(debugMove);

    if (!moveResult)
    {
        std::cerr << moveResult.message << '\n';
        return false;
    }
#endif

    return true;
}


void ViewerApp::InitPhysics(const Entity& robotRoot, Entity& floorEntity)
{
    m_PhysicsWorld = std::make_unique<PhysicsWorld>();
    m_PhysicsSystemModule = std::make_unique<PhysicsSystemModule>(m_World, *m_PhysicsWorld);

    EntityFactory::ConfigureFloor(floorEntity);
    EntityFactory::ConfigureRobotPhysics(robotRoot, grasplink::robotics::models::hanwha::kHcr12a);
#ifndef NDEBUG
    // Debug 전용: Dynamic Cube 시각화
    EntityFactory::CreateDebugBox(*m_SceneManager->GetActiveScene());
#endif
}


void ViewerApp::MainLoop()
{
    using Clock = std::chrono::steady_clock;

    auto lastFrameTime = Clock::now();

    while (!m_Window->ShouldClose())
    {
        // 이전 화면 갱신 이후 경과 시간 (초)
        const auto currentFrameTime = Clock::now();
        const double frameDeltaSeconds = std::chrono::duration<double>(currentFrameTime - lastFrameTime).count();

        lastFrameTime = currentFrameTime;

        // 디버거 정지 뒤 긴 시간값이 한꺼번에 반영되는 상황 방지
        const double clampedFrameDeltaSeconds = std::min(frameDeltaSeconds, kMaxFrameDeltaSeconds);
        const float renderDeltaSeconds = static_cast<float>(clampedFrameDeltaSeconds);

        // Window 이벤트·Camera 입력
        m_Window->PollEvents();
        m_CameraController->OnUpdate();

        // 고정 Simulation 순서: Controller → Entity 자세 → Physics
        // 창 최소화 여부와 무관하게 실행
        m_ControlLoop.Advance(frameDeltaSeconds, [this](double fixedDeltaSeconds)
        {
            m_RobotController->Update(fixedDeltaSeconds);
            m_RobotTransformAdapter->Apply(m_RobotController->GetState());
            m_PhysicsSystemModule->Step(fixedDeltaSeconds);
        });

        // 최소화 상태 (Framebuffer 가로·세로가 0): Render 건너뜀
        int framebufferWidth = 0;
        int framebufferHeight = 0;

        m_Window->GetFramebufferSize(framebufferWidth, framebufferHeight);

        if (framebufferWidth <= 0 || framebufferHeight <= 0)
            continue;

        // 변경된 Framebuffer 크기 반영
        m_Renderer->Resize(framebufferWidth, framebufferHeight);

        const float aspectRatio = static_cast<float>(framebufferWidth) / static_cast<float>(framebufferHeight);
        m_Camera->SetAspectRatio(aspectRatio);

        // Flecs 진행: Transform 갱신 후 Render System 실행
        m_Renderer->BeginFrame();

        m_SceneManager->OnUpdate(renderDeltaSeconds);
        m_World.progress(renderDeltaSeconds);

        m_Renderer->EndFrame();
        m_Window->SwapBuffers();
    }
}


void ViewerApp::Shutdown()
{
    // 참조 객체부터 제거
    m_RobotTransformAdapter.reset();

    // Robot 연결 종료
    if (m_RobotController)
        m_RobotController->Disconnect();

    m_RobotController.reset();

    // Scene Entity 정리 → Physics Body observer 처리
    m_SceneManager.reset();
    m_PhysicsSystemModule.reset();

    // Entity 제거가 끝난 뒤 PhysicsWorld 파괴
    m_World.reset();
    m_PhysicsWorld.reset();

    m_AssetManager.reset();

    // Viewer Resource 역순 정리
    m_CameraController.reset();
    m_Camera.reset();
    m_Renderer.reset();
    m_Window.reset();
}
