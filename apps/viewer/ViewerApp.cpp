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
#include "gui/panels/GripperPanel.h"
#include "gui/panels/PhysicsDebugPanel.h"
#include "gui/overlays/ColliderOverlay.h"
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

// Viewer 창은 시작할 때 이 픽셀 크기로 요청한다. 실제 렌더 대상 크기는 DPI 배율에 따라 framebuffer에서 다시 읽는다.
constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 720;

// 로봇 제어기와 물리 시뮬레이션은 화면 프레임률과 관계없이 250 Hz, 즉 4 ms 간격으로 갱신한다.
constexpr double kControlFrequencyHz = 250.0;
constexpr double kControlFixedDeltaSeconds = 1.0 / kControlFrequencyHz;

// 창 이동이나 디버거 정지 뒤 긴 시간이 쌓여도 한 프레임에서 최대 100 ms만 시뮬레이션에 누적한다.
constexpr double kMaxFrameDeltaSeconds = 0.1;

const char* kWindowTitle = "GraspLink Viewer";

// 카메라의 시작 위치와 바라볼 지점은 모두 Scene의 World 좌표 [m]로 지정한다.
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
    // 먼저 OpenGL Context를 만들고, 그 Context를 사용하는 Scene과 GPU 모델, 로봇 계산기, Physics Body 순서로 연결한다.
    // 중간 단계가 실패해도 Run()의 종료 경로가 Shutdown()을 호출해 그때까지 만들어진 자원을 정리한다.
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

    // Renderer는 논리 창 크기가 아니라 DPI 배율이 반영된 실제 framebuffer 크기로 초기화해야 픽셀과 화면이 맞는다.
    m_Renderer = std::make_unique<Renderer>();
    m_Renderer->Init(framebufferWidth, framebufferHeight);

    const float aspectRatio = static_cast<float>(kWindowWidth) / static_cast<float>(kWindowHeight);
    m_Camera = std::make_unique<Camera>(kCameraPosition, kCameraTarget, aspectRatio);
    m_GuiModule = std::make_unique<grasplink::gui::GuiModule>(*m_Window);
    m_GripperPanel = std::make_unique<grasplink::gui::GripperPanel>();
    m_PhysicsDebugPanel = std::make_unique<grasplink::gui::PhysicsDebugPanel>();
    m_ColliderOverlay = std::make_unique<grasplink::gui::ColliderOverlay>(m_World);

    m_CameraController = std::make_unique<OrbitCameraController>(*m_Camera, *m_Window);

    // Flecs World에 Renderer와 Camera의 포인터를 전달하지만 이 자료는 World가 소유하지 않는다.
    // Shutdown에서는 이 포인터를 읽는 system과 World를 두 객체보다 먼저 정리한다.
    m_World.set<RenderContext>({m_Renderer.get(), m_Camera.get()});

    // Transform은 fixed step의 물리 전후와 렌더 직전에 계산한다. RenderSystem은 World 진행 시 렌더 대상을 처리한다.
    m_World.import<TransformSystemModule>();
    m_World.import<RenderSystemModule>();

    m_SceneManager = std::make_unique<SceneManager>(m_World);
    m_SceneManager->LoadScene<Scene>();

    // 예약한 Scene을 활성화해 계층의 최상위 부모를 만든 뒤, Scene 소유 Entity로 모델 계층을 구성한다.
    m_SceneManager->OnUpdate(0.0F);

    m_AssetManager = std::make_unique<AssetManager>();

    return true;
}


void ViewerApp::InitScene(Entity& robotRoot, Entity& floorEntity)
{
    Scene* scene = m_SceneManager->GetActiveScene();

    m_RobotShader = Shader::Create("shaders/Robot.glsl");
    auto gridShader = Shader::Create("shaders/Grid.glsl");

    // GLB를 읽어 CPU 모델 자료를 만든 다음 Mesh와 Material을 OpenGL 자원으로 업로드한다.
    m_RobotModel = GltfLoader::LoadGLB("HCR12A_2F-85.glb");
    m_AssetManager->UploadModel(m_RobotModel);

    // GLB의 node 부모 관계를 Flecs Entity 계층으로 복사한다. 반환한 robotRoot 아래에서 이후 J1~J6 관절을 찾는다.
    robotRoot = PrefabFactory::CreateModel(
        *scene,
        m_RobotModel,
        *m_AssetManager,
        m_RobotShader);

    // 바닥 Mesh를 만들고, InitPhysics()에서 같은 Entity에 움직이지 않는 Static Collider를 연결한다.
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
    // 로봇 모델 사양에서 관절 수, 회전축, 허용 범위와 최대 속도를 가져온다.
    const auto& robotSpec = grasplink::robotics::models::hanwha::kHcr12a;

    auto simController = std::make_unique<SimRobotController>(robotSpec);

    const auto connectResult = simController->Connect();

    if (!connectResult)
    {
        std::cerr << connectResult.message << '\n';
        return false;
    }

    m_RobotController = std::move(simController);

    // 관절 각도로 각 링크의 위치와 회전을 계산하는 정기구학(Forward Kinematics) 결과를 화면 Entity와 충돌용 단순 형상에도 적용해 두 자세가 어긋나지 않게 한다.
    m_RobotKinematics = std::make_unique<grasplink::robotics::kinematics::RobotKinematics>(robotSpec);
    m_RobotTransformAdapter = std::make_unique<RobotTransformAdapter>(robotRoot, robotSpec);

    // 자동 관절 이동은 빌드 종류로 결정하지 않고 사용자가 --physics-demo를 지정했을 때만 시작한다.
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

    // 데모와 smoke test에서는 테스트할 수 있도록 그리퍼를 실제로 닫는다. 일반 Viewer는 열린 기준 자세에서 시작한다.
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
    // 한 고정 tick에서 팔의 관절 자세와 그리퍼의 부모 기준 Local 회전을 함께 적용한다. 이어지는 World 변환 갱신으로 화면과 충돌 프록시가 같은 계층 자세를 얻는다.
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
    // 화면에는 원본 GLB Mesh를 사용하고, 물리에는 각 정점에서 방향별 극점을 골라 단순화한 볼록 껍질(Convex Hull) 충돌 형상을 사용한다.
    // 바깥 관절(outer knuckle)과 손가락처럼 같은 강체 부품에 속한 Mesh만 합쳐 복합 충돌 형상 하나로 만든다.
    // 그리퍼 본체와 여섯 관절에 Scene이 목표 자세를 정하는 Kinematic Body 설정을 만든다.
    // 충돌용 단순 형상은 원본 관절 Entity의 자식으로 두어 그 관절을 따라가게 한다.
    // GripperState에서 계산한 Local 회전을 원본 관절에 적용한 뒤 World 행렬을 갱신하면 자식인 충돌용 단순 형상도 함께 움직인다.
    // 각 충돌 형상은 원본 관절의 자식이므로 따로 움직이는 adapter가 필요하지 않다. 현재 그리퍼 물리는 접촉 형상만 준비하며 접촉에 따른 정지와 파지는 구현하지 않는다.
    // GUI의 보라색 선은 ECS에 지정한 shape를 깊이 가림 없이 그린 근사다. 화면 Mesh나 Jolt가 최종 생성한 hull을 직접 보여 주지는 않는다.
    grasplink::simulation::ConfigureTwoF85Colliders(
        *m_SceneManager->GetActiveScene(), robotRoot, m_RobotModel);
    if (m_Options.physicsDemo)
        viewer_debug::CreatePhysicsBoxes(
            *m_SceneManager->GetActiveScene(), m_RobotShader);
    // Physics Body를 만들기 전에 첫 FK 자세와 계층 World 행렬을 계산해 화면 Entity와 Kinematic 목표를 같은 위치에 맞춘다.
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

        // 디버거 정지 등으로 한 프레임이 길어져도 설정한 최대 시간만 제어 및 물리 누적기에 전달한다.
        const double clampedFrameDeltaSeconds = std::min(frameDeltaSeconds, kMaxFrameDeltaSeconds);
        const float renderDeltaSeconds = static_cast<float>(clampedFrameDeltaSeconds);

        // 창 이벤트를 처리하고, ImGui가 마우스를 사용하지 않을 때만 카메라 입력을 전달한다.
        m_Window->PollEvents();
        if (!m_GuiModule->WantsMouse())
            m_CameraController->OnUpdate();

        // 고정 갱신에서는 Controller 상태를 읽어 FK를 계산하고 Entity 자세와 World 행렬을 만든 뒤 Jolt를 진행한다.
        // PhysicsSystem은 Kinematic Body의 목표 자세를 Jolt에 보내고, Dynamic Body가 계산한 결과를 ECS Local 값으로 되돌린다.
        // 물리 step 뒤 World 행렬을 다시 계산해야 다음 렌더가 부모와 자식의 최신 자세를 사용한다. 창이 최소화되어도 이 시뮬레이션 갱신은 계속된다.
        m_ControlLoop.Advance(frameDeltaSeconds, [this](double fixedDeltaSeconds)
        {
            m_RobotController->Update(fixedDeltaSeconds);
            m_GripperController->Update(fixedDeltaSeconds);
            ApplyControllerPoses();
            TransformSystemModule::UpdateWorldTransforms(m_World);
            m_PhysicsSystemModule->Step(fixedDeltaSeconds);
            TransformSystemModule::UpdateWorldTransforms(m_World);
        });

        // 최소화된 창은 framebuffer의 가로 또는 세로가 0일 수 있으므로, 이때 GPU 렌더링 단계만 건너뛴다.
        int framebufferWidth = 0;
        int framebufferHeight = 0;

        m_Window->GetFramebufferSize(framebufferWidth, framebufferHeight);

        if (framebufferWidth <= 0 || framebufferHeight <= 0)
            continue;

        m_Renderer->Resize(framebufferWidth, framebufferHeight);

        const float aspectRatio = static_cast<float>(framebufferWidth) / static_cast<float>(framebufferHeight);
        m_Camera->SetAspectRatio(aspectRatio);

        // 화면 주기 Scene 갱신과 World 행렬 계산을 마친 뒤 ECS가 저장한 collider 설정과 자세를 GUI overlay가 읽는다.
        m_Renderer->BeginFrame();

        m_SceneManager->OnUpdate(renderDeltaSeconds);
        TransformSystemModule::UpdateWorldTransforms(m_World);
        m_World.progress(renderDeltaSeconds);
        m_GuiModule->BeginFrame();
        m_GripperPanel->Draw(*m_GripperController);
        m_PhysicsDebugPanel->Draw();
        m_ColliderOverlay->Draw(*m_Camera, m_PhysicsDebugPanel->IsColliderVisible());
        m_GuiModule->EndFrame();

        m_Renderer->EndFrame();
        m_Window->SwapBuffers();
        if (m_Options.smokeTest && ++renderedFrames >= 8)
            break;
    }
}


void ViewerApp::Shutdown()
{
    // ECS query를 가진 overlay와 패널을 World보다 먼저 해제한다. 패널은 Controller를 소유하지 않으므로 Controller보다 먼저 끝내도 된다.
    m_ColliderOverlay.reset();
    m_PhysicsDebugPanel.reset();
    m_GripperPanel.reset();
    // Scene Entity handle을 빌린 Adapter를 Scene보다 먼저 파괴해 소멸 처리 중 이미 삭제된 handle을 참조하지 않게 한다.
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

    // Scene을 파괴하면 ECS 삭제 observer가 Jolt Body를 제거한다.
    // 따라서 SceneManager를 정리하는 동안 물리 시스템과 Jolt World를 유지한다.
    m_SceneManager.reset();
    m_PhysicsSystemModule.reset();
    m_GuiModule.reset();

    // 모든 Scene Body가 제거된 뒤 observer와 ECS World를 해제하고, 마지막으로 Jolt PhysicsWorld를 파괴한다.
    m_World.reset();
    m_PhysicsWorld.reset();

    // ModelResource가 GPU Mesh를 공유하므로 마지막 참조를 OpenGL Context가 살아 있을 때 해제해야 Mesh 소멸자가 안전하게 동작한다.
    m_RobotModel = {};
    m_AssetManager.reset();
    m_RobotShader.reset();

    // GUI와 GPU 객체를 모두 정리한 다음 Window를 파괴해 OpenGL Context를 마지막에 닫는다.
    m_CameraController.reset();
    m_Camera.reset();
    m_Renderer.reset();
    m_Window.reset();
}
