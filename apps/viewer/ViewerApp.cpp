#include "ViewerApp.h"

#include "ViewerRobotCollisionGuard.h"

#include "Camera.h"
#include "PickPlaceScenario.h"
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
#include "gui/panels/RobotPanel.h"
#include "gui/panels/PhysicsDebugPanel.h"
#include "gui/overlays/ColliderOverlay.h"
#include "simulation/SimulationSceneBuilder.h"
#include "simulation/RandomScenario.h"
#include "simulation/robotics/GripperColliders.h"
#include "simulation/robotics/GripperGraspAdapter.h"
#include "simulation/robotics/RobotPhysicsAdapter.h"
#include "simulation/systems/PhysicsSystemModule.h"

#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/backends/simulation/SimGripperController.h"
#include "robotics/models/hanwha/Hcr12a.h"
#include "robotics/models/robotiq/TwoF85.h"
#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/kinematics/GripperKinematics.h"

#include "scene/Scene.h"

#include "viewer/robotics/RobotTransformAdapter.h"
#include "viewer/robotics/GripperTransformAdapter.h"

#include <algorithm>
#include <cmath>
#include <glm/gtc/quaternion.hpp>
#include <imgui.h>
#include <chrono>
#include <memory>
#include <limits>
#include <stdexcept>
#if GRASPLINK_ENABLE_TRACY
#include <tracy/Tracy.hpp>
#else
#define ZoneScopedN(name) ((void)0)
#define FrameMark ((void)0)
#endif

namespace
{

// Viewer 창은 시작할 때 이 픽셀 크기로 요청한다. 실제 렌더 대상 크기는 DPI 배율에 따라 framebuffer에서 다시 읽는다.
constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 720;
constexpr float kControlSidebarWidth = 390.0F;

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

// World 행렬에는 Entity의 크기 변경도 들어가므로 축 길이를 제거한 뒤 회전만 quaternion으로 바꾼다.
glm::quat RotationFromWorldMatrix(const glm::mat4& worldMatrix)
{
    glm::mat3 axes(worldMatrix);
    for (int axis = 0; axis < 3; ++axis)
        axes[axis] = glm::normalize(axes[axis]);
    return glm::normalize(glm::quat_cast(axes));
}

/**
 * @brief 열린 손끝 메시 두 개의 중심 사이를 Viewer에서 사용할 고정 TCP로 정한다.
 * @details ToolFrame은 공구 장착면의 기준점이고 TCP는 공구에서 이동 목표를 지정할 작업점이다.
 * 메시의 꼭짓점을 Gripper 기준 좌표로 옮겨 각 메시를 감싸는 상자의 중심을 구하고, 두 중심의 중점을 사용한다.
 * 이 선택은 현재 GLB를 위한 명시적인 시뮬레이션 기준이며 제조사 TCP 보정값이 아니다. 손가락을 닫아도 TCP 보정값은 바꾸지 않는다.
 * TCP의 방향은 Gripper의 축 방향으로 정하고, 위치와 방향을 ToolFrame 기준으로 바꿔 Controller에 전달한다.
 */
grasplink::robotics::models::Pose3 CalculateViewerTcp(const Entity& robotRoot, const ModelResource& model)
{
    const Entity tool = robotRoot.FindChildByNameRecursive("ToolFrame");
    const Entity gripper = robotRoot.FindChildByNameRecursive("Gripper");
    if (!tool.IsValid() || !gripper.IsValid())
        throw std::runtime_error("ViewerApp: missing TCP reference frames");
    const glm::mat4 worldToGripper = glm::inverse(gripper.GetWorldMatrix());
    glm::vec3 midpoint(0.0F);
    for (const char* name : {"LeftFingerTipMesh", "RightFingerTipMesh"})
    {
        const auto node = std::find_if(model.nodes.begin(), model.nodes.end(), [name](const NodeData& value) { return value.name == name; });
        const Entity meshEntity = robotRoot.FindChildByNameRecursive(name);
        if (node == model.nodes.end() || !meshEntity.IsValid() || node->meshIndex < 0 || static_cast<std::size_t>(node->meshIndex) >= model.meshes.size())
            throw std::runtime_error("ViewerApp: missing TCP fingertip geometry");
        const auto& vertices = model.meshes[static_cast<std::size_t>(node->meshIndex)].vertices;
        if (vertices.empty())
            throw std::runtime_error("ViewerApp: empty TCP fingertip geometry");
        glm::vec3 minimum(std::numeric_limits<float>::max());
        glm::vec3 maximum(std::numeric_limits<float>::lowest());
        const glm::mat4 meshToGripper = worldToGripper * meshEntity.GetWorldMatrix();
        for (const auto& vertex : vertices)
        {
            const glm::vec3 point(meshToGripper * glm::vec4(vertex.position, 1.0F));
            minimum = glm::min(minimum, point);
            maximum = glm::max(maximum, point);
        }
        midpoint += (minimum + maximum) * 0.25F;
    }
    const glm::mat4 gripperToTool = glm::inverse(tool.GetWorldMatrix()) * gripper.GetWorldMatrix();
    const glm::vec3 position(gripperToTool * glm::vec4(midpoint, 1.0F));
    const glm::quat orientation = glm::normalize(glm::quat_cast(glm::mat3(gripperToTool)));
    return {{position.x, position.y, position.z}, {orientation.w, orientation.x, orientation.y, orientation.z}};
}

} // namespace


ViewerApp::ViewerApp(ViewerOptions options)
    : m_Options(options),
      m_ControlLoop(kControlFixedDeltaSeconds, kMaxFrameDeltaSeconds)
{
}


ViewerApp::~ViewerApp()
{
    Shutdown();
}


int ViewerApp::Run()
{
    try
    {
        if (!Init())
        {
            m_Logger.Write(grasplink::diagnostics::LogLevel::Error, "viewer.init", "Viewer 초기화에 실패했습니다.");
            Shutdown();
            return -1;
        }

        MainLoop();
        Shutdown();
        return 0;
    }
    catch (const std::exception& error)
    {
        try
        {
            m_Logger.Write(grasplink::diagnostics::LogLevel::Error, "viewer.run", error.what());
        }
        catch (...)
        {
        }
        Shutdown();
        throw;
    }
    catch (...)
    {
        try
        {
            m_Logger.Write(grasplink::diagnostics::LogLevel::Error, "viewer.run", "알 수 없는 예외가 발생했습니다.");
        }
        catch (...)
        {
        }
        Shutdown();
        throw;
    }
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
    m_RobotRoot = std::make_unique<Entity>(robotRoot);

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
    m_RobotPanel = std::make_unique<grasplink::gui::RobotPanel>();
    m_PhysicsDebugPanel = std::make_unique<grasplink::gui::PhysicsDebugPanel>();
    m_ColliderOverlay = std::make_unique<grasplink::gui::ColliderOverlay>(m_World);

    m_CameraController = std::make_unique<OrbitCameraController>(*m_Camera, *m_Window);

    // Flecs World에 Renderer와 Camera의 포인터를 전달하지만 이 자료는 World가 소유하지 않는다.
    // Shutdown에서는 이 포인터를 읽는 system과 World를 두 객체보다 먼저 정리한다.
    m_World.set<RenderContext>({m_Renderer.get(), m_Camera.get()});

    // Transform은 fixed step의 물리 전후와 렌더 직전에 계산한다. RenderSystem은 World 진행 시 렌더 대상을 처리한다.
    m_World.import<TransformSystemModule>();
    m_World.import<RenderSystemModule>();

    m_Scene = std::make_unique<Scene>(m_World);

    m_AssetManager = std::make_unique<AssetManager>();

    return true;
}


void ViewerApp::InitScene(Entity& robotRoot, Entity& floorEntity)
{
    Scene* scene = m_Scene.get();

    m_RobotShader = Shader::Create("shaders/Robot.glsl");
    auto gridShader = Shader::Create("shaders/Grid.glsl");

    // GLB를 읽어 CPU 모델 자료를 만든 다음 Mesh와 Material을 OpenGL 자원으로 업로드한다.
    m_RobotModel = GltfLoader::LoadGLB("HCR12A_2F-85.glb");
    m_AssetManager->UploadModel(m_RobotModel);

    // GLB의 node 부모 관계를 Flecs Entity 계층으로 복사한다. 반환한 robotRoot 아래에서 이후 J1~J6 관절을 찾는다.
    robotRoot = prefab_factory::CreateModel(
        *scene,
        m_RobotModel,
        *m_AssetManager,
        m_RobotShader);

    // 바닥 Mesh를 만들고, InitPhysics()에서 같은 Entity에 움직이지 않는 Static Collider를 연결한다.
    ModelResource planeModel = GltfLoader::LoadGLB("plane.glb");
    m_AssetManager->UploadModel(planeModel);

    floorEntity = prefab_factory::CreateModel(
        *scene,
        planeModel,
        *m_AssetManager,
        gridShader);

}


bool ViewerApp::InitRobot(const Entity& robotRoot)
{
    // 로봇 모델 사양에서 관절 수, 회전축, 허용 범위와 최대 속도를 가져온다.
    const auto& robotSpec = grasplink::robotics::models::hanwha::kHcr12a;

    // 모델 계층이 보관한 Local 변환을 합친 뒤 TCP를 정한다. World 변환의 역행렬을 사용하므로 로봇 전체 배치는 TCP 보정에 포함되지 않는다.
    TransformSystemModule::UpdateWorldTransforms(m_World);
    auto simController = std::make_unique<SimRobotController>(robotSpec, CalculateViewerTcp(robotRoot, m_RobotModel));

    const auto connectResult = simController->Connect();

    if (!connectResult)
    {
        m_Logger.Write(grasplink::diagnostics::LogLevel::Error, "robot.connect", connectResult.message);
        return false;
    }

    m_RobotController = std::move(simController);

    // 관절 각도로 각 링크의 위치와 회전을 계산하는 정기구학(Forward Kinematics) 결과를 화면 Entity와 충돌용 단순 형상에도 적용해 두 자세가 어긋나지 않게 한다.
    m_RobotKinematics = std::make_unique<grasplink::robotics::kinematics::RobotKinematics>(robotSpec);
    m_RobotTransformAdapter = std::make_unique<RobotTransformAdapter>(robotRoot, robotSpec);

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
        m_Logger.Write(grasplink::diagnostics::LogLevel::Error, "gripper.initialize", result.message);
        return false;
    }
    m_GripperKinematics = std::make_unique<grasplink::robotics::kinematics::GripperKinematics>(specification);
    m_GripperTransformAdapter = std::make_unique<grasplink::viewer::robotics::GripperTransformAdapter>(
        robotRoot.FindChildByNameRecursive("Gripper"), specification);

    // 데모와 smoke test에서는 테스트할 수 있도록 그리퍼를 실제로 닫는다. 일반 Viewer는 열린 기준 자세에서 시작한다.
    if (m_Options.smokeTest)
    {
        const auto result = m_GripperController->Command({255, 255, 128});
        if (!result)
        {
            m_Logger.Write(grasplink::diagnostics::LogLevel::Error, "gripper.smoke_test_command", result.message);
            return false;
        }
    }
    return true;
}


void ViewerApp::ApplyControllerPoses()
{
    const auto& robotPose = m_RobotKinematics->Update(m_RobotController->GetStateView());
    m_RobotTransformAdapter->Apply(robotPose);
    m_RobotPhysicsAdapter->Apply(robotPose);
    // 한 고정 tick에서 팔의 관절 자세와 그리퍼의 부모 기준 Local 회전을 함께 적용한다. 이어지는 World 변환 갱신으로 화면과 충돌 프록시가 같은 계층 자세를 얻는다.
    const auto& gripperPose = m_GripperKinematics->Update(m_GripperController->GetState());
    m_GripperTransformAdapter->Apply(gripperPose);
}


void ViewerApp::InitPhysics(const Entity& robotRoot, Entity& floorEntity)
{
    m_PhysicsWorld = std::make_unique<PhysicsWorld>();
    grasplink::simulation::ConfigureFloor(floorEntity);
    m_RobotPhysicsAdapter = std::make_unique<grasplink::simulation::RobotPhysicsAdapter>(
        *m_Scene, robotRoot, grasplink::robotics::models::hanwha::kHcr12a,
        m_RobotModel);
    // 화면에는 원본 GLB Mesh를 사용하고, 물리에는 각 정점에서 방향별 극점을 골라 단순화한 볼록 껍질(Convex Hull) 충돌 형상을 사용한다.
    // 바깥 관절(outer knuckle)과 손가락처럼 같은 강체 부품에 속한 Mesh만 합쳐 복합 충돌 형상 하나로 만든다.
    // 그리퍼 본체와 여섯 관절에 Scene이 목표 자세를 정하는 Kinematic Body 설정을 만든다.
    // 충돌용 단순 형상은 원본 관절 Entity의 자식으로 두어 그 관절을 따라가게 한다.
    // GripperState에서 계산한 Local 회전을 원본 관절에 적용한 뒤 World 행렬을 갱신하면 자식인 충돌용 단순 형상도 함께 움직인다.
    // 각 충돌 형상은 원본 관절의 자식이므로 관절 자세를 전달하는 별도 adapter는 필요하지 않다. 접촉 정지와 물체 파지는 아래 GripperGraspAdapter가 물리 계산 결과를 받아 처리한다.
    // GUI의 보라색 선은 ECS에 지정한 shape를 깊이 가림 없이 그린 근사다. 화면 Mesh나 Jolt가 최종 생성한 hull을 직접 보여 주지는 않는다.
    grasplink::simulation::ConfigureTwoF85Colliders(
        *m_Scene, robotRoot, m_RobotModel);
    m_GraspBox = std::make_unique<Entity>(grasplink::viewer::pick_place::CreateGraspBox(*m_Scene, m_RobotShader));
    m_PlacementArea = std::make_unique<Entity>(grasplink::viewer::pick_place::CreatePlacementArea(*m_Scene, m_RobotShader));
    // Physics Body를 만들기 전에 첫 FK 자세와 계층 World 행렬을 계산해 화면 Entity와 Kinematic 목표를 같은 위치에 맞춘다.
    ApplyControllerPoses();
    TransformSystemModule::UpdateWorldTransforms(m_World);
    m_PhysicsSystemModule = std::make_unique<grasplink::simulation::PhysicsSystemModule>(
        m_World, *m_PhysicsWorld, &m_Logger);
    m_GripperGraspAdapter = std::make_unique<grasplink::simulation::GripperGraspAdapter>(
        *m_PhysicsWorld, *m_PhysicsSystemModule,
        *m_GripperController);
    if (!m_GripperGraspAdapter->Bind(robotRoot))
        throw std::runtime_error("ViewerApp: cannot bind gripper contact bodies");

    m_RobotCollisionGuard = std::make_unique<grasplink::viewer::ViewerRobotCollisionGuard>(
        *m_RobotController, *m_GripperController,
        *m_RobotKinematics, *m_RobotTransformAdapter, *m_RobotPhysicsAdapter,
        *m_GripperKinematics, *m_GripperTransformAdapter, m_World, *m_PhysicsWorld,
        *m_PhysicsSystemModule, robotRoot);
}


void ViewerApp::MainLoop()
{
    using Clock = std::chrono::steady_clock;

    auto lastFrameTime = Clock::now();
    std::size_t renderedFrames = 0;

    while (!m_Window->ShouldClose())
    {
        ZoneScopedN("Frame");
        FrameMark;
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
            ZoneScopedN("FixedTick");
            {
                // 열기·Reset·연결 해제 요청은 물리 계산 전에 파지 제약을 없애야 물체가 다음 계산부터 자유롭게 떨어진다.
                m_GripperGraspAdapter->BeforePhysicsStep();
                m_RobotCollisionGuard->CaptureSafeJointPose();
                {
                    ZoneScopedN("RobotUpdate");
                    m_RobotController->Update(fixedDeltaSeconds);
                    m_GripperController->Update(fixedDeltaSeconds);
                    ApplyControllerPoses();
                }
                TransformSystemModule::UpdateWorldTransforms(m_World);
                m_RobotCollisionGuard->RestoreSafePoseIfOverlapping();
                {
                    ZoneScopedN("PhysicsStep");
                    m_PhysicsSystemModule->Step(fixedDeltaSeconds);
                }
                // Jolt가 이번 간격의 접촉을 모두 계산한 뒤 그리퍼에 결과를 돌려준다. 접촉 정지는 다음 고정 갱신부터 개폐 진행을 막는다.
                m_GripperGraspAdapter->AfterPhysicsStep();
                TransformSystemModule::UpdateWorldTransforms(m_World);
            }
        });

        // 최소화된 창은 framebuffer의 가로 또는 세로가 0일 수 있으므로, 이때 GPU 렌더링 단계만 건너뛴다.
        int framebufferWidth = 0;
        int framebufferHeight = 0;

        m_Window->GetFramebufferSize(framebufferWidth, framebufferHeight);

        if (framebufferWidth <= 0 || framebufferHeight <= 0)
            continue;

        m_Renderer->Resize(framebufferWidth, framebufferHeight);
        m_GuiModule->BeginFrame();

        const ImGuiIO& guiIo = ImGui::GetIO();
        const float displayWidth = std::max(guiIo.DisplaySize.x, 1.0F);
        const float displayHeight = std::max(guiIo.DisplaySize.y, 1.0F);
        const float sidebarWidth = std::min(kControlSidebarWidth, displayWidth * 0.42F);
        const float horizontalScale = static_cast<float>(framebufferWidth) / displayWidth;
        const int sceneWidth = std::clamp(
            static_cast<int>(std::lround((displayWidth - sidebarWidth) * horizontalScale)),
            1, framebufferWidth);
        const float aspectRatio = static_cast<float>(sceneWidth) / static_cast<float>(framebufferHeight);
        m_Camera->SetAspectRatio(aspectRatio);
        m_Renderer->SetSceneViewport(0, 0, sceneWidth, framebufferHeight);

        // Scene은 화면 왼쪽 viewport에 그린다. 오른쪽 ImGui sidebar는 같은 창 위에 고정 배치한다.
        {
            ZoneScopedN("Render");
            m_Renderer->BeginFrame();

            TransformSystemModule::UpdateWorldTransforms(m_World);
            m_World.progress(renderDeltaSeconds);
            const auto graspState = m_GripperGraspAdapter->GetState();
            const glm::mat4 robotBaseWorld = m_RobotRoot->GetWorldMatrix();
            const glm::mat4 worldToRobotBase = glm::inverse(robotBaseWorld);
            const glm::quat worldToRobotBaseRotation = glm::inverse(RotationFromWorldMatrix(robotBaseWorld));
            const auto toRobotBasePose = [&](const glm::mat4& worldMatrix, float heightOffset)
            {
                const glm::vec3 position(worldToRobotBase * worldMatrix[3]);
                const glm::quat orientation = glm::normalize(
                    worldToRobotBaseRotation * RotationFromWorldMatrix(worldMatrix));
                return grasplink::robotics::CartesianPose{
                    {position.x, position.y + heightOffset, position.z},
                    {orientation.x, orientation.y, orientation.z, orientation.w}};
            };
            const auto boxPose = toRobotBasePose(m_GraspBox->GetWorldMatrix(), 0.0F);
            const auto placementPose = toRobotBasePose(m_PlacementArea->GetWorldMatrix(), 0.020F);
            const float sidebarX = displayWidth - sidebarWidth;
            ImGui::SetNextWindowPos(ImVec2(sidebarX, 0.0F), ImGuiCond_Always);
            ImGui::SetNextWindowSize(ImVec2(sidebarWidth, displayHeight), ImGuiCond_Always);
            constexpr ImGuiWindowFlags sidebarFlags = ImGuiWindowFlags_NoTitleBar |
                ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse |
                ImGuiWindowFlags_NoSavedSettings;
            ImGui::Begin("Robot controls", nullptr, sidebarFlags);
            ImGui::TextUnformatted("GraspLink | HCR-12A");
            ImGui::Separator();
            const auto& stateForMission = m_RobotController->GetStateView();
            m_PickPlaceMission.Update(stateForMission, *m_RobotController, *m_GripperController,
                graspState.grasped, boxPose, placementPose);
            const auto& robotState = m_RobotController->GetStateView();
            const auto mission = m_PickPlaceMission.Snapshot();
            const grasplink::gui::RobotPanelMissionView missionView{mission.stageLabel, mission.lastMessage,
                mission.completedCount, 2.0, 8.0, mission.paused, mission.missionSucceeded,
                mission.autoRepeat, mission.hasResult, mission.lastRequestAccepted, mission.canStart};
            const grasplink::gui::RobotPanelView robotPanelView{robotState,
                m_RobotController->GetSpecification(), boxPose, placementPose, missionView};
            const auto panelActions = m_RobotPanel->DrawContents(robotPanelView);
            m_PickPlaceMission.ApplyActions({panelActions.start, panelActions.resume, panelActions.stop},
                m_RobotController->GetStateView(), *m_RobotController, graspState.grasped, boxPose);
            if (m_PickPlaceMission.ConsumeSuccessEvent())
            {
                const auto nextPosition = grasplink::simulation::scenario::SampleBoxPosition();
                const auto nextGoalPosition = grasplink::simulation::scenario::SamplePlacementPosition();
                m_GraspBox->SetLocalPosition({nextPosition[0], nextPosition[1], nextPosition[2]});
                m_GraspBox->SetLocalRotation(glm::angleAxis(
                    grasplink::simulation::scenario::SamplePlanarRotation(), glm::vec3{0.0F, 1.0F, 0.0F}));
                m_PlacementArea->SetLocalPosition({nextGoalPosition[0], nextGoalPosition[1], nextGoalPosition[2]});
                m_PlacementArea->SetLocalRotation(glm::angleAxis(
                    grasplink::simulation::scenario::SamplePlanarRotation(), glm::vec3{0.0F, 1.0F, 0.0F}));
                TransformSystemModule::UpdateWorldTransforms(m_World);
                const glm::mat4 boxTransform = m_GraspBox->GetWorldMatrix();
                const auto boxHandle = m_PhysicsSystemModule->GetBodyHandle(m_GraspBox->GetHandle());
                const glm::quat boxRotation = RotationFromWorldMatrix(boxTransform);
                m_PhysicsWorld->SetBodyTransform(boxHandle, grasplink::physics::Transform{
                    glm::vec3(boxTransform[3]), boxRotation});
                m_GripperGraspAdapter->Release();
                m_PickPlaceMission.PrepareNextTask();
            }
            m_GripperPanel->DrawContents(*m_GripperController, &graspState);
            m_PhysicsDebugPanel->DrawContents();
            ImGui::End();
            m_ColliderOverlay->Draw(*m_Camera, m_PhysicsDebugPanel->IsColliderVisible(),
                ImVec2(displayWidth - sidebarWidth, displayHeight));
            m_Renderer->EndFrame();
            m_GuiModule->EndFrame();
            m_Window->SwapBuffers();
        }
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
    m_RobotPanel.reset();
    // 파지 제약은 연결된 Body와 PhysicsWorld를 빌려 쓰므로 Scene, Controller, 물리 시스템이 살아 있을 때 먼저 해제한다.
    m_GripperGraspAdapter.reset();
    // Controller가 검증 함수를 보관하므로 guard 소멸자가 참조를 해제할 때까지 Controller가 살아 있어야 한다.
    m_RobotCollisionGuard.reset();
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
    // Destroy the Scene while physics observers and the Jolt world are alive.
    m_RobotRoot.reset();
    m_GraspBox.reset();
    m_PlacementArea.reset();
    m_Scene.reset();
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

    m_Logger.Flush();
    m_Logger.Shutdown();
}
