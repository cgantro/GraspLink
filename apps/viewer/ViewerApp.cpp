#include "ViewerApp.h"

#include "Camera.h"
#include "DebugSceneSetup.h"
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

#include "PhysicsWorld.h"
#include "gui/GuiModule.h"
#include "simulation/SimulationSceneBuilder.h"
#include "simulation/robotics/GripperColliders.h"
#include "simulation/robotics/RobotPhysicsAdapter.h"
#include "simulation/systems/PhysicsSystemModule.h"

#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/backends/simulation/SimGripperController.h"
#include "robotics/core/IRobotController.h"
#include "robotics/core/IGripperController.h"
#include "robotics/models/hanwha/Hcr12a.h"
#include "robotics/models/robotiq/TwoF85.h"
#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/kinematics/GripperKinematics.h"

#include "scene/Scene.h"
#include "scene/SceneManager.h"

#include "viewer/robotics/RobotTransformAdapter.h"
#include "viewer/robotics/GripperTransformAdapter.h"

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

using SimRobotController = grasplink::robotics::backends::simulation::SimRobotController;
using RobotTransformAdapter = grasplink::viewer::robotics::RobotTransformAdapter;
using PhysicsWorld = grasplink::physics::PhysicsWorld;

} // namespace


ViewerApp::ViewerApp(ViewerOptions options)
    : m_Options(options), m_ControlLoop(kControlFixedDeltaSeconds, kMaxFrameDeltaSeconds)
{
}


ViewerApp::~ViewerApp()
{
    Shutdown();
}


int ViewerApp::Run()
{
    if (!Init())
        return -1;

    MainLoop();
    return 0;
}


bool ViewerApp::Init()
{
    // 초기화 순서: Context가 필요한 Viewer → GPU 모델/Entity Scene → Controller와 FK → Physics Body.
    // 각 단계 실패도 Run의 종료 경로에서 Shutdown이 부분 생성 자원을 정리한다.
    if (!InitViewer())
        return false;

    Entity robotRoot;
    Entity floorEntity;
    InitScene(robotRoot, floorEntity);

    if (!InitRobot(robotRoot))
        return false;

    if (!InitGripper(robotRoot))
        return false;

    InitPhysics(robotRoot, floorEntity);

    return true;
}


bool ViewerApp::InitViewer()
{
    // Window가 OpenGL Context를 소유한다. Renderer와 AssetManager의 GPU 자원은 이 Context 안에서 만든다.
    m_Window = std::make_unique<Window>(
        Window::Properties{kWindowWidth, kWindowHeight, kWindowTitle, !m_Options.smokeTest, !m_Options.smokeTest});

    int framebufferWidth = 0;
    int framebufferHeight = 0;
    m_Window->GetFramebufferSize(framebufferWidth, framebufferHeight);

    // DPI 배율이 반영된 실제 Framebuffer 크기로 Renderer 초기화
    m_Renderer = std::make_unique<Renderer>();
    m_Renderer->Init(framebufferWidth, framebufferHeight);

    const float aspectRatio = static_cast<float>(kWindowWidth) / static_cast<float>(kWindowHeight);
    m_Camera = std::make_unique<Camera>(kCameraPosition, kCameraTarget, aspectRatio);
    m_GuiModule = std::make_unique<grasplink::gui::GuiModule>(
        m_World, *m_Window, *m_Camera);

    m_CameraController = std::make_unique<OrbitCameraController>(*m_Camera, *m_Window);

    // World context는 소유하지 않고 포인터를 빌린다. Shutdown에서 World와 참조 객체를 알맞은 순서로 없앤다.
    m_World.set<RenderContext>({m_Renderer.get(), m_Camera.get()});

    // Transform은 fixed step의 물리 전후와 렌더 직전에 계산한다. RenderSystem은 World 진행 시 렌더 대상을 처리한다.
    m_World.import<TransformSystemModule>();
    m_World.import<RenderSystemModule>();

    m_SceneManager = std::make_unique<SceneManager>(m_World);
    m_SceneManager->LoadScene<Scene>();

    // 예약 Scene을 활성화해 root와 계층을 만든 뒤, 이 Entity 소유 Scene 안에 모델 Entity를 생성한다.
    m_SceneManager->OnUpdate(0.0F);

    m_AssetManager = std::make_unique<AssetManager>();

    return true;
}


void ViewerApp::InitScene(Entity& robotRoot, Entity& floorEntity)
{
    Scene* scene = m_SceneManager->GetActiveScene();

    m_RobotShader = Shader::Create("shaders/Robot.glsl");
    auto gridShader = Shader::Create("shaders/Grid.glsl");

    // GLB 읽기 → GPU Mesh·Material 업로드
    m_RobotModel = GltfLoader::LoadGLB("HCR12A_2F-85.glb");
    m_AssetManager->UploadModel(m_RobotModel);

    // GLB Node 계층 → Flecs Entity 계층
    // robotRoot: 이후 J1~J6 검색 기준
    robotRoot = PrefabFactory::CreateModel(
        *scene,
        m_RobotModel,
        *m_AssetManager,
        m_RobotShader);

    // Floor Mesh 생성. Static Collider는 InitPhysics에서 같은 Entity에 연결
    ModelResource planeModel = GltfLoader::LoadGLB("plane.glb");
    m_AssetManager->UploadModel(planeModel);

    floorEntity = PrefabFactory::CreateModel(
        *scene,
        planeModel,
        *m_AssetManager,
        gridShader);

}


bool ViewerApp::InitRobot(const Entity& robotRoot)
{
    // HCR-12A 관절 수·축·제한·최대 속도
    const auto& robotSpec = grasplink::robotics::models::hanwha::kHcr12a;

    auto simController = std::make_unique<SimRobotController>(robotSpec);

    const auto connectResult = simController->Connect();

    if (!connectResult)
    {
        std::cerr << connectResult.message << '\n';
        return false;
    }

    m_RobotController = std::move(simController);

    // 같은 FK 결과를 렌더 Entity와 Kinematic collider Entity에 전달해 충돌 프록시가 렌더 자세를 따른다.
    m_RobotKinematics = std::make_unique<grasplink::robotics::kinematics::RobotKinematics>(robotSpec);
    m_RobotTransformAdapter = std::make_unique<RobotTransformAdapter>(robotRoot, robotSpec);

    // 데모는 빌드 종류와 무관하게 명시적 실행 옵션으로만 시작한다.
    if (m_Options.physicsDemo)
    {
        const auto moveResult = viewer_debug::StartRobotMotion(*m_RobotController, robotSpec);
        if (!moveResult)
        {
            std::cerr << moveResult.message << '\n';
            return false;
        }
    }

    return true;
}


bool ViewerApp::InitGripper(const Entity& robotRoot)
{
    const auto& specification = grasplink::robotics::models::robotiq::kTwoF85;
    m_GripperController = std::make_unique<grasplink::robotics::backends::simulation::SimGripperController>(specification);
    auto result = m_GripperController->Connect();
    if (result)
        result = m_GripperController->Activate();
    if (!result)
    {
        std::cerr << result.message << '\n';
        return false;
    }
    m_GripperKinematics = std::make_unique<grasplink::robotics::kinematics::GripperKinematics>(specification);
    m_GripperTransformAdapter = std::make_unique<grasplink::viewer::robotics::GripperTransformAdapter>(
        robotRoot.FindChildByNameRecursive("Gripper"), specification);

    // 명시한 데모와 숨김 실행은 실제 개폐를 진행한다. 일반 실행은 열린 기준 자세에서 조작한다.
    if (m_Options.physicsDemo || m_Options.smokeTest)
    {
        const auto result = m_GripperController->Command({255, 255, 128});
        if (!result)
        {
            std::cerr << result.message << '\n';
            return false;
        }
    }
    return true;
}


void ViewerApp::ApplyControllerPoses()
{
    const auto& robotPose = m_RobotKinematics->Update(m_RobotController->GetState());
    m_RobotTransformAdapter->Apply(robotPose);
    m_RobotPhysicsAdapter->Apply(robotPose);
    // 팔의 부모 자세와 그리퍼의 Local 회전을 같은 tick에 반영해 화면·충돌 프록시가 같은 계층을 따른다.
    const auto& gripperPose = m_GripperKinematics->Update(m_GripperController->GetState());
    m_GripperTransformAdapter->Apply(gripperPose);
}


void ViewerApp::InitPhysics(const Entity& robotRoot, Entity& floorEntity)
{
    m_PhysicsWorld = std::make_unique<PhysicsWorld>();
    grasplink::simulation::SimulationSceneBuilder::ConfigureFloor(floorEntity);
    m_RobotPhysicsAdapter = std::make_unique<grasplink::simulation::RobotPhysicsAdapter>(
        *m_SceneManager->GetActiveScene(), robotRoot, grasplink::robotics::models::hanwha::kHcr12a,
        m_RobotModel);
    // 렌더 Mesh는 원본 GLB 형상, Physics는 정점의 방향별 극점으로 축약한 Convex Hull 설정을 사용한다.
    // outer knuckle과 finger처럼 한 rigid part에 묶인 메시만 compound로 합쳐 접촉 형상을 만든다.
    // Gripper 본체와 6개 관절 프록시, 총 7개 Kinematic Body 설정을 만든다. 각 proxy는 authored joint Entity의 자식이다.
    // GripperState에서 계산한 Local 회전을 원본 관절에 적용하면 World 갱신이 자식 proxy의 자세도 만든다.
    // 프록시를 별도로 구동하는 adapter는 필요하지 않다. 접촉 시 정지·파지는 후속 물리 작업이다.
    // GUI 보라색 선은 ECS 설정 shape를 그린 X-ray 근사이며 렌더 Mesh/Jolt가 실제 생성한 hull의 시각화가 아니다.
    grasplink::simulation::ConfigureTwoF85Colliders(
        *m_SceneManager->GetActiveScene(), robotRoot, m_RobotModel);
    if (m_Options.physicsDemo)
        viewer_debug::CreatePhysicsBoxes(
            *m_SceneManager->GetActiveScene(), m_RobotShader);
    // Body 생성 전에 초기 FK와 World 행렬을 준비해 첫 Kinematic 목표와 렌더 Entity를 맞춘다.
    ApplyControllerPoses();
    TransformSystemModule::UpdateWorldTransforms(m_World);
    m_PhysicsSystemModule = std::make_unique<grasplink::simulation::PhysicsSystemModule>(m_World, *m_PhysicsWorld);
}


void ViewerApp::MainLoop()
{
    using Clock = std::chrono::steady_clock;

    auto lastFrameTime = Clock::now();
    std::size_t renderedFrames = 0;

    while (!m_Window->ShouldClose())
    {
        const auto currentFrameTime = Clock::now();
        const double frameDeltaSeconds = m_Options.smokeTest ? 1.0 / 60.0
            : std::chrono::duration<double>(currentFrameTime - lastFrameTime).count();

        lastFrameTime = currentFrameTime;

        // 디버거 정지 뒤 긴 시간값이 한꺼번에 반영되는 상황 방지
        const double clampedFrameDeltaSeconds = std::min(frameDeltaSeconds, kMaxFrameDeltaSeconds);
        const float renderDeltaSeconds = static_cast<float>(clampedFrameDeltaSeconds);

        // Window 이벤트·Camera 입력
        m_Window->PollEvents();
        if (!m_GuiModule->WantsMouse())
            m_CameraController->OnUpdate();

        // 고정 순서: Controller 상태 → FK → 시각/물리 Entity → World 행렬 → Jolt step.
        // PhysicsSystem은 Kinematic 목표를 제출하고 Dynamic Body 결과를 ECS Local로 반영한다.
        // step 뒤 World 행렬을 다시 계산해 다음 렌더에서 부모/자식 Entity 자세가 맞게 한다. 최소화 중에도 시뮬레이션은 돈다.
        m_ControlLoop.Advance(frameDeltaSeconds, [this](double fixedDeltaSeconds)
        {
            m_RobotController->Update(fixedDeltaSeconds);
            m_GripperController->Update(fixedDeltaSeconds);
            ApplyControllerPoses();
            TransformSystemModule::UpdateWorldTransforms(m_World);
            m_PhysicsSystemModule->Step(fixedDeltaSeconds);
            TransformSystemModule::UpdateWorldTransforms(m_World);
        });

        // 최소화 상태 (Framebuffer 가로·세로가 0): Render 건너뜀
        int framebufferWidth = 0;
        int framebufferHeight = 0;

        m_Window->GetFramebufferSize(framebufferWidth, framebufferHeight);

        if (framebufferWidth <= 0 || framebufferHeight <= 0)
            continue;

        m_Renderer->Resize(framebufferWidth, framebufferHeight);

        const float aspectRatio = static_cast<float>(framebufferWidth) / static_cast<float>(framebufferHeight);
        m_Camera->SetAspectRatio(aspectRatio);

        // 렌더 주기 Scene logic과 최신 World 행렬을 반영한다. GUI는 이 ECS 설정/자세를 별도 갱신 주기로 읽는다.
        m_Renderer->BeginFrame();

        m_SceneManager->OnUpdate(renderDeltaSeconds);
        TransformSystemModule::UpdateWorldTransforms(m_World);
        m_World.progress(renderDeltaSeconds);
        m_GuiModule->Draw(m_GripperController.get());

        m_Renderer->EndFrame();
        m_Window->SwapBuffers();
        if (m_Options.smokeTest && ++renderedFrames >= 8)
            break;
    }
}


void ViewerApp::Shutdown()
{
    // Entity handle을 빌리는 Adapter를 Scene보다 먼저 정리한다. 그렇지 않으면 소멸 과정에서 stale handle이 된다.
    m_RobotTransformAdapter.reset();
    m_GripperTransformAdapter.reset();
    m_RobotPhysicsAdapter.reset();
    m_RobotKinematics.reset();
    m_GripperKinematics.reset();

    if (m_RobotController)
        m_RobotController->Disconnect();

    m_RobotController.reset();
    if (m_GripperController)
        m_GripperController->Disconnect();
    m_GripperController.reset();

    // Scene 정리의 ECS observer가 Jolt Body를 제거할 수 있도록 PhysicsSystemModule과 PhysicsWorld를 아직 살려 둔다.
    m_SceneManager.reset();
    m_PhysicsSystemModule.reset();
    m_GuiModule.reset();

    // Scene의 Body 정리가 끝난 다음 observer/query를 해제하고 World, 이어 Jolt PhysicsWorld를 정리한다.
    m_World.reset();
    m_PhysicsWorld.reset();

    // ModelResource도 GPU Mesh를 공유한다. 마지막 참조는 OpenGL Context보다 먼저 해제한다.
    m_RobotModel = {};
    m_AssetManager.reset();
    m_RobotShader.reset();

    // GUI와 모든 GPU 리소스 정리 뒤 마지막으로 OpenGL Context를 파괴한다.
    m_CameraController.reset();
    m_Camera.reset();
    m_Renderer.reset();
    m_Window.reset();
}
