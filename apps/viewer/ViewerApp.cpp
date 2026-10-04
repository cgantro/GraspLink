#include "ViewerApp.h"

#include "Camera.h"
#include "Entity.h"
#include "Material.h"
#include "Mesh.h"
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
#include "components/RenderComponents.h"
#include "components/TransformComponents.h"

#include "PhysicsWorld.h"
#include "systems/PhysicsSyncSystem.h"

#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/core/IRobotController.h"
#include "robotics/models/hanwha/Hcr12a.h"

#include "scene/Scene.h"
#include "scene/SceneManager.h"

#include "viewer/robotics/RobotTransformAdapter.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>

namespace
{

// Viewer 창 크기.
constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 720;

// Robot / Physics Simulation을 초당 250번 갱신한다.
// 1 / 250 = 0.004초이므로 Fixed Update 한 번의 dt는 항상 4ms다.
constexpr double kControlFrequencyHz = 250.0;
constexpr double kControlFixedDeltaSeconds = 1.0 / kControlFrequencyHz;

// 디버거 정지나 창 이동 등으로 frame 시간이 크게 튀었을 때
// 한 프레임에 지나치게 많은 Simulation Update가 실행되는 것을 막는다.
constexpr double kMaxFrameDeltaSeconds = 0.1;

const char* kWindowTitle = "GraspLink Viewer";

// Viewer 시작 시 Camera 위치와 바라보는 지점.
const glm::vec3 kCameraPosition{2.0F, 1.35F, 1.15F};
const glm::vec3 kCameraTarget{0.05F, 0.50F, 0.40F};

// 긴 namespace를 반복하지 않기 위한 별칭.
using SimRobotController = grasplink::robotics::backends::simulation::SimRobotController;
using RobotTransformAdapter = grasplink::viewer::robotics::RobotTransformAdapter;

using PhysicsWorld = grasplink::physics::PhysicsWorld;
using PhysicsBodyHandle = grasplink::physics::PhysicsBodyHandle;
using BoxBodyDescription = grasplink::physics::BoxBodyDescription;
using BodyMotionType = grasplink::physics::BodyMotionType;

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
    // 모든 Runtime 객체 생성에 실패하면 실행하지 않는다.
    if (!Init())
        return -1;

    MainLoop();
    return 0;
}


bool ViewerApp::Init()
{
    /*
     * 초기화 책임을 단계별로 분리한다.
     *
     * 1. Window / Renderer / ECS 준비
     * 2. Scene과 모델 생성
     * 3. Robot Controller 연결
     * 4. Physics World 생성
     */
    if (!InitViewer())
        return false;

    Entity robotRoot = InitScene();

    if (!InitRobot(robotRoot))
        return false;

    InitPhysics();

    return true;
}


bool ViewerApp::InitViewer()
{
    /*
     * 1. Window 생성.
     *
     * Window가 OpenGL Context도 함께 소유한다.
     */
    m_Window = std::make_unique<Window>(
        Window::Properties{kWindowWidth, kWindowHeight, kWindowTitle, true});

    int framebufferWidth = 0;
    int framebufferHeight = 0;
    m_Window->GetFramebufferSize(framebufferWidth, framebufferHeight);

    /*
     * 2. Renderer 초기화.
     *
     * 실제 Framebuffer 크기를 사용해야
     * DPI scaling이 적용된 환경에서도 올바른 크기로 렌더링된다.
     */
    m_Renderer = std::make_unique<Renderer>();
    m_Renderer->Init(framebufferWidth, framebufferHeight);

    /*
     * 3. Camera 생성.
     *
     * aspectRatio = 화면 가로 / 세로 비율.
     */
    const float aspectRatio = static_cast<float>(kWindowWidth) / static_cast<float>(kWindowHeight);
    m_Camera = std::make_unique<Camera>(kCameraPosition, kCameraTarget, aspectRatio);

    // Mouse 입력을 Orbit / Pan / Zoom으로 변환한다.
    m_CameraController = std::make_unique<OrbitCameraController>(*m_Camera, *m_Window);

    /*
     * 4. Renderer와 Camera를 Flecs World에 등록한다.
     *
     * RenderSystem이 별도 전역 변수 없이
     * 현재 Renderer와 Camera에 접근하기 위한 Runtime Context다.
     */
    m_World.set<RenderContext>({m_Renderer.get(), m_Camera.get()});

    /*
     * 5. ECS System 등록.
     *
     * TransformSystem:
     * Local Transform -> World Transform 계산
     *
     * RenderSystem:
     * 렌더 가능한 Entity를 찾아 Renderer에 전달
     */
    m_World.import<TransformSystemModule>();
    m_World.import<RenderSystemModule>();

    /*
     * 6. 기본 Scene 생성.
     */
    m_SceneManager = std::make_unique<SceneManager>(m_World);
    m_SceneManager->LoadScene<Scene>();

    // Scene이 처음 생성된 직후 필요한 초기 Update를 한 번 수행한다.
    m_SceneManager->OnUpdate(0.0F);

    /*
     * 7. Mesh / Texture 등의 GPU Resource를 관리할 AssetManager 생성.
     */
    m_AssetManager = std::make_unique<AssetManager>();

    return true;
}


Entity ViewerApp::InitScene()
{
    Scene* scene = m_SceneManager->GetActiveScene();

    /*
     * Robot과 Floor는 사용하는 Shader가 다르다.
     */
    auto robotShader = Shader::Create("shaders/Robot.glsl");
    auto gridShader = Shader::Create("shaders/Grid.glsl");

    /*
     * 1. HCR-12A + Robotiq 2F-85 모델 로드.
     *
     * GLB를 CPU 데이터로 읽은 뒤
     * AssetManager를 통해 GPU Resource를 생성한다.
     */
    ModelResource robotModel = GltfLoader::LoadGLB("HCR12A_2F-85.glb");
    m_AssetManager->UploadModel(robotModel);

    /*
     * PrefabFactory가 GLB Node hierarchy를
     * Flecs Entity hierarchy로 생성한다.
     *
     * 반환되는 robotRoot는 이후
     * RobotTransformAdapter가 J1~J6 Entity를 찾는 시작점이다.
     */
    Entity robotRoot = PrefabFactory::CreateModel(
        *scene,
        robotModel,
        *m_AssetManager,
        robotShader);

    /*
     * 2. 화면에 표시할 Grid Plane 생성.
     *
     * 이 Plane은 현재 Jolt Physics Floor와는 별개의 객체다.
     *
     * plane.glb:
     * 화면 표시용
     *
     * PhysicsWorld의 Static Box:
     * 실제 충돌 계산용
     */
    ModelResource planeModel = GltfLoader::LoadGLB("plane.glb");
    m_AssetManager->UploadModel(planeModel);

    PrefabFactory::CreateModel(
        *scene,
        planeModel,
        *m_AssetManager,
        gridShader);

    return robotRoot;
}


bool ViewerApp::InitRobot(const Entity& robotRoot)
{
    /*
     * HCR-12A의 관절 수, 회전축, 각도 제한,
     * 최대 속도 등이 들어 있는 Robot Specification.
     */
    const auto& robotSpec = grasplink::robotics::models::hanwha::kHcr12a;

    /*
     * 1. 현재는 실제 Robot Hardware 대신
     * Simulation Controller를 사용한다.
     */
    auto simController = std::make_unique<SimRobotController>(robotSpec);

    const auto connectResult = simController->Connect();

    if (!connectResult)
    {
        std::cerr << connectResult.message << '\n';
        return false;
    }

    /*
     * 이후 코드는 구체적인 SimRobotController가 아니라
     * IRobotController 인터페이스를 통해 Robot을 제어한다.
     */
    m_RobotController = std::move(simController);

    /*
     * 2. Robot Controller 상태와 Viewer 모델을 연결한다.
     *
     * RobotState의 J1~J6 각도
     *        ↓
     * RobotTransformAdapter
     *        ↓
     * GLB Joint Entity의 Local Rotation
     */
    m_RobotTransformAdapter = std::make_unique<RobotTransformAdapter>(robotRoot, robotSpec);

    /*
     * 3. Robot Controller 동작 확인용 임시 명령.
     *
     * 모든 관절을 0 rad로 두고
     * J1만 30도로 이동시킨다.
     *
     * 향후 UI / Trajectory / IK 명령으로 교체한다.
     */
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

    return true;
}


void ViewerApp::InitPhysics()
{
    /*
     * 1. Jolt Physics World 생성.
     *
     * PhysicsWorld 내부에서:
     * - Jolt 초기화
     * - 중력 설정
     * - Collision Layer
     * - PhysicsSystem
     * - JobSystem
     *
     * 등을 관리한다.
     */
    m_PhysicsWorld = std::make_unique<PhysicsWorld>();

    /*
     * PhysicsSyncSystem은 Physics 계산을 하지 않는다.
     *
     * Jolt Body Transform
     *        ↓
     * Flecs Entity Transform
     *
     * 사이의 데이터를 연결하는 역할이다.
     */
    m_PhysicsSyncSystem = std::make_unique<PhysicsSyncSystem>(m_World, *m_PhysicsWorld);

    /*
     * 2. 충돌용 Static Floor 생성.
     *
     * halfExtents는 실제 크기의 절반이다.
     *
     * 전체 크기:
     * X = 10m
     * Y = 0.2m
     * Z = 10m
     *
     * 중심 Y가 -0.1m이므로
     * Floor의 윗면은 정확히 Y = 0m에 위치한다.
     */
    BoxBodyDescription floor;
    floor.halfExtentsMeters = {5.0F, 0.1F, 5.0F};
    floor.transform.position = {0.0F, -0.1F, 0.0F};
    floor.motionType = BodyMotionType::Static;

    m_PhysicsWorld->CreateBox(floor);

    /*
     * 3. 낙하 테스트용 Dynamic Box 생성.
     *
     * halfExtent = 0.25m이므로 실제 크기는:
     *
     * 0.5m x 0.5m x 0.5m
     *
     * 중심을 Y = 2m에서 시작시키면
     * 중력에 의해 떨어진 후 Floor 위에서
     * 중심 Y ≈ 0.25m에 정지해야 한다.
     */
    BoxBodyDescription box;
    box.halfExtentsMeters = {0.25F, 0.25F, 0.25F};
    box.transform.position = {0.0F, 2.0F, 0.0F};
    box.motionType = BodyMotionType::Dynamic;

    m_DebugBoxBody = m_PhysicsWorld->CreateBox(box);

    /*
     * 4. Physics Body와 연결할 Flecs Entity 생성.
     *
     * Scene::CreateEntity()를 사용하므로
     * Position / Rotation / Scale / TransformMatrix가 자동으로 붙는다.
     */
    Scene* scene = m_SceneManager->GetActiveScene();
    Entity debugBoxEntity = scene->CreateEntity("PhysicsDebugBox");

    /*
     * 이 Component가:
     *
     * Flecs Entity
     *      ↓
     * PhysicsBodyHandle
     *      ↓
     * Jolt Body
     *
     * 관계를 만든다.
     */
    PhysicsBodyComponent physicsBody;
    physicsBody.body = m_DebugBoxBody;
    physicsBody.syncMode = PhysicsSyncMode::PhysicsToEntity;

    debugBoxEntity.Set(physicsBody);

    /*
     * 현재 debugBoxEntity에는 Render Component가 없기 때문에
     * Physics 위치는 ECS에 반영되지만 화면에는 아직 Box가 보이지 않는다.
     *
     * 다음 단계에서 Cube Mesh를 붙여 직접 낙하를 확인한다.
     */
}


void ViewerApp::MainLoop()
{
    using Clock = std::chrono::steady_clock;

    auto lastFrameTime = Clock::now();

    while (!m_Window->ShouldClose())
    {
        /*
         * 1. 이전 Render Frame 이후 실제로 흐른 시간을 계산한다.
         *
         * 이 값은 모니터 주사율이나 GPU 성능에 따라 계속 달라질 수 있다.
         */
        const auto currentFrameTime = Clock::now();
        const double frameDeltaSeconds = std::chrono::duration<double>(currentFrameTime - lastFrameTime).count();

        lastFrameTime = currentFrameTime;

        /*
         * 너무 큰 frame dt는 0.1초로 제한한다.
         *
         * 디버거 정지 후 복귀 같은 상황에서
         * Scene / Rendering에 매우 큰 dt가 전달되는 것을 막는다.
         */
        const double clampedFrameDeltaSeconds = std::min(frameDeltaSeconds, kMaxFrameDeltaSeconds);
        const float renderDeltaSeconds = static_cast<float>(clampedFrameDeltaSeconds);

        /*
         * 2. Window Event와 Camera 입력 처리.
         */
        m_Window->PollEvents();
        m_CameraController->OnUpdate();

        /*
         * 3. 현재 Framebuffer 크기를 확인한다.
         *
         * 창이 최소화되면 width 또는 height가 0이 될 수 있으므로
         * 그 상태에서는 Rendering을 건너뛴다.
         */
        int framebufferWidth = 0;
        int framebufferHeight = 0;

        m_Window->GetFramebufferSize(framebufferWidth, framebufferHeight);

        if (framebufferWidth <= 0 || framebufferHeight <= 0)
            continue;

        /*
         * Window 크기가 변했다면 Renderer와 Camera 비율을 갱신한다.
         */
        m_Renderer->Resize(framebufferWidth, framebufferHeight);

        const float aspectRatio = static_cast<float>(framebufferWidth) / static_cast<float>(framebufferHeight);
        m_Camera->SetAspectRatio(aspectRatio);

        /*
         * 4. Fixed Simulation Update.
         *
         * Render FPS와 관계없이 Robot / Physics는
         * 항상 0.004초씩 일정하게 진행된다.
         *
         * 예:
         *
         * frame dt = 0.016초라면
         * Fixed Update가 대략 4번 실행된다.
         */
        m_ControlLoop.Advance(frameDeltaSeconds, [this](double fixedDeltaSeconds)
        {
            // Robot 관절 상태를 한 Fixed Step 진행한다.
            m_RobotController->Update(fixedDeltaSeconds);

            // 중력, 충돌, Dynamic Body 이동을 한 Fixed Step 계산한다.
            m_PhysicsWorld->Step(fixedDeltaSeconds);
        });

        /*
         * 5. Physics 결과를 Flecs Transform에 반영한다.
         *
         * Jolt Dynamic Body Transform
         *          ↓
         * PhysicsBodyComponent
         *          ↓
         * Position / Rotation(Local)
         */
        m_PhysicsSyncSystem->SyncPhysicsToEntity();

        /*
         * 6. Robot Controller의 최신 Joint 상태를
         * Robot GLB hierarchy에 반영한다.
         */
        m_RobotTransformAdapter->Apply(m_RobotController->GetState());

        /*
         * 7. Rendering.
         *
         * m_World.progress() 안에서 TransformSystem이 실행되어
         * 방금 변경된 Local Transform을 World Matrix로 계산한다.
         */
        m_Renderer->BeginFrame();

        m_SceneManager->OnUpdate(renderDeltaSeconds);
        m_World.progress(renderDeltaSeconds);

        m_Renderer->EndFrame();
        m_Window->SwapBuffers();
    }
}


void ViewerApp::Shutdown()
{
    /*
     * 참조 관계가 남지 않도록
     * 생성의 역순에 가깝게 Runtime 객체를 제거한다.
     */

    // RobotTransformAdapter가 Scene Entity와 Robot 정보를 참조하므로 먼저 제거한다.
    m_RobotTransformAdapter.reset();

    // Controller가 연결되어 있으면 먼저 정상 종료한다.
    if (m_RobotController)
        m_RobotController->Disconnect();

    m_RobotController.reset();

    /*
     * PhysicsSyncSystem은 PhysicsWorld를 참조한다.
     * 따라서 반드시 PhysicsWorld보다 먼저 제거한다.
     */
    m_PhysicsSyncSystem.reset();

    // Handle은 Body를 소유하지 않으므로 invalid 상태로 되돌린다.
    m_DebugBoxBody = PhysicsBodyHandle{};

    // 실제 Jolt Body와 PhysicsSystem은 PhysicsWorld가 소유한다.
    m_PhysicsWorld.reset();

    /*
     * Scene / Asset / ECS 제거.
     */
    m_SceneManager.reset();
    m_AssetManager.reset();
    m_World.reset();

    /*
     * Viewer Resource 제거.
     */
    m_CameraController.reset();
    m_Camera.reset();
    m_Renderer.reset();
    m_Window.reset();
}