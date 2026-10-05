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
#include "simulation/robotics/RobotPhysicsAdapter.h"
#include "simulation/systems/PhysicsSystemModule.h"

#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/core/IRobotController.h"
#include "robotics/models/hanwha/Hcr12a.h"
#include "robotics/kinematics/RobotKinematics.h"

#include "scene/Scene.h"
#include "scene/SceneManager.h"

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

    // RenderSystem이 사용할 Renderer·Camera 등록
    m_World.set<RenderContext>({m_Renderer.get(), m_Camera.get()});

    // Transform은 fixed step과 렌더 직전에 명시적으로 계산하고, RenderSystem은 World 진행 때 실행한다.
    m_World.import<TransformSystemModule>();
    m_World.import<RenderSystemModule>();

    m_SceneManager = std::make_unique<SceneManager>(m_World);
    m_SceneManager->LoadScene<Scene>();

    // 예약 Scene을 활성화해 root를 만든 뒤 모델 Entity를 생성한다.
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

    // 같은 FK 결과를 시각 Entity와 Kinematic collider Entity에 각각 전달한다.
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


void ViewerApp::InitPhysics(const Entity& robotRoot, Entity& floorEntity)
{
    m_PhysicsWorld = std::make_unique<PhysicsWorld>();
    grasplink::simulation::SimulationSceneBuilder::ConfigureFloor(floorEntity);
    m_RobotPhysicsAdapter = std::make_unique<grasplink::simulation::RobotPhysicsAdapter>(
        *m_SceneManager->GetActiveScene(), robotRoot, grasplink::robotics::models::hanwha::kHcr12a,
        m_RobotModel);
    if (m_Options.physicsDemo)
        viewer_debug::CreatePhysicsBoxes(
            *m_SceneManager->GetActiveScene(), m_RobotShader);
    // Body 생성 전에 초기 FK와 World 행렬을 준비한다. 첫 Jolt pose가 화면 Entity와 일치해야 한다.
    const auto& robotPose = m_RobotKinematics->Update(m_RobotController->GetState());
    m_RobotTransformAdapter->Apply(robotPose);
    m_RobotPhysicsAdapter->Apply(robotPose);
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
        // step 뒤 Dynamic 결과가 Local로 돌아오므로 World 행렬을 다시 계산한다. 최소화 중에도 실행한다.
        m_ControlLoop.Advance(frameDeltaSeconds, [this](double fixedDeltaSeconds)
        {
            m_RobotController->Update(fixedDeltaSeconds);
            const auto& robotPose = m_RobotKinematics->Update(m_RobotController->GetState());
            m_RobotTransformAdapter->Apply(robotPose);
            m_RobotPhysicsAdapter->Apply(robotPose);
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

        // 렌더 직전 Scene logic과 World 행렬을 반영한다. GUI는 갱신 주기에 맞춰 이 상태를 읽는다.
        m_Renderer->BeginFrame();

        m_SceneManager->OnUpdate(renderDeltaSeconds);
        TransformSystemModule::UpdateWorldTransforms(m_World);
        m_World.progress(renderDeltaSeconds);
        m_GuiModule->Draw();

        m_Renderer->EndFrame();
        m_Window->SwapBuffers();
        if (m_Options.smokeTest && ++renderedFrames >= 8)
            break;
    }
}


void ViewerApp::Shutdown()
{
    // Entity handle을 빌리는 Adapter를 Scene보다 먼저 정리한다.
    m_RobotTransformAdapter.reset();
    m_RobotPhysicsAdapter.reset();
    m_RobotKinematics.reset();

    if (m_RobotController)
        m_RobotController->Disconnect();

    m_RobotController.reset();

    // Scene 삭제 observer가 Jolt Body를 제거할 때 PhysicsSystem과 PhysicsWorld가 살아 있어야 한다.
    m_SceneManager.reset();
    m_PhysicsSystemModule.reset();
    m_GuiModule.reset();

    // GUI query와 물리 observer를 먼저 해제한 뒤 World와 PhysicsWorld를 정리한다.
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
