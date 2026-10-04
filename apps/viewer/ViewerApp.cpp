#include "ViewerApp.h"

#include "Camera.h"
#include "OrbitCameraController.h"
#include "Material.h"
#include "Mesh.h"
#include "RenderContext.h"
#include "Renderer.h"
#include "RenderSystemModule.h"
#include "Shader.h"
#include "TransformSystemModule.h"
#include "Window.h"

#include "viewer/robotics/RobotTransformAdapter.h"

#include "assets/GltfLoader.h"
#include "assets/AssetManager.h"
#include "assets/PrefabFactory.h"

#include "components/RenderComponents.h"
#include "components/TransformComponents.h"

#include "scene/Scene.h"
#include "scene/SceneManager.h"

#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/core/IRobotController.h"
#include "robotics/models/hanwha/Hcr12a.h"

#include <chrono>
#include <iostream>
#include <memory>

namespace
{
constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 720;
const char* kWindowTitle = "GraspLink Viewer";

const glm::vec3 kCameraPosition{2.0F, 1.35F, 1.15F};
const glm::vec3 kCameraTarget{0.05F, 0.50F, 0.40F};
} // namespace

ViewerApp::ViewerApp() = default;

ViewerApp::~ViewerApp()
{
    Shutdown();
}

int ViewerApp::Run()
{
    if (!Init()) return -1;

    MainLoop();
    return 0;
}

bool ViewerApp::Init()
{
    m_Window = std::make_unique<Window>(
        Window::Properties{kWindowWidth, kWindowHeight, kWindowTitle, true});

    int framebufferWidth = 0;
    int framebufferHeight = 0;
    m_Window->GetFramebufferSize(framebufferWidth, framebufferHeight);

    m_Renderer = std::make_unique<Renderer>();
    m_Renderer->Init(framebufferWidth, framebufferHeight);

    m_Camera = std::make_unique<Camera>(
        kCameraPosition,
        kCameraTarget,
        static_cast<float>(kWindowWidth) / static_cast<float>(kWindowHeight));

    m_CameraController = std::make_unique<OrbitCameraController>(
        *m_Camera,
        *m_Window);

    m_World.set<RenderContext>({m_Renderer.get(), m_Camera.get()});
    m_World.import<TransformSystemModule>();
    m_World.import<RenderSystemModule>();

    m_SceneManager = std::make_unique<SceneManager>(m_World);
    m_SceneManager->LoadScene<Scene>();
    m_SceneManager->OnUpdate(0.0F);

    Scene* scene = m_SceneManager->GetActiveScene();

    m_AssetManager = std::make_unique<AssetManager>();

    auto robotShader = Shader::Create("shaders/Robot.glsl");
    auto gridShader = Shader::Create("shaders/Grid.glsl");

    /*
     * GLB는 로봇의 시각 geometry와 Joint hierarchy를 제공한다.
     * 로봇의 angle limit/max velocity 같은 제어 사양은 GLB가 아니라 RobotSpecification에서 가져온다.
     */
    ModelResource robotModel = GltfLoader::LoadGLB("HCR12A_2F-85.glb");
    m_AssetManager->UploadModel(robotModel);

    Entity robotRoot = PrefabFactory::CreateModel(
        *scene,
        robotModel,
        *m_AssetManager,
        robotShader);

    ModelResource planeModel = GltfLoader::LoadGLB("plane.glb");
    m_AssetManager->UploadModel(planeModel);
    Entity planeRoot = PrefabFactory::CreateModel(
        *scene,
        planeModel,
        *m_AssetManager,
        gridShader);
    (void)planeRoot;

    /*
     * HCR-12A의 모델 사양(source of truth)을 선택한다.
     * 이 값에는 J1~J6 이름, pivot, axis, angle limit, max velocity가 들어 있다.
     */
    const auto& robotSpec = grasplink::robotics::models::hanwha::kHcr12a;

    /*
     * SimRobotController는 실제 모터를 제어하지 않는다.
     * RobotSpecification을 이용해 메모리 속 q(관절각)/dq(관절속도)를 시간에 따라 갱신한다.
     */
    auto simController =
        std::make_unique<grasplink::robotics::backends::simulation::SimRobotController>(robotSpec);

    const grasplink::robotics::Result connectResult = simController->Connect();
    if (!connectResult)
    {
        std::cerr << connectResult.message << '\n';
        return false;
    }

    /*
     * 상위 계층에서는 구체적인 SimRobotController 타입 대신 IRobotController로 보관한다.
     * 나중에 실제 Hardware backend로 교체해도 MainLoop의 호출 형태를 유지하기 위한 구조다.
     */
    m_RobotController = std::move(simController);

    /*
     * RobotTransformAdapter는 "제어"가 아니라 "표현" 담당이다.
     * RobotState의 q[rad]를 같은 이름의 GLB Joint Entity local rotation으로 바꾼다.
     */
    m_RobotTransformAdapter =
        std::make_unique<grasplink::viewer::robotics::RobotTransformAdapter>(robotRoot, robotSpec);

    /*
     * 임시 검증 command:
     * Joint-space 명령 배열을 J1..J6 순서로 만들고 J1의 절대 목표각만 30도로 지정한다.
     * glm::radians(30.0)은 30 degree를 공통 runtime 단위인 radian으로 변환한다.
     *
     * 향후 입력/UI/trajectory 계층으로 교체할 테스트 코드다.
     */
    grasplink::robotics::JointMoveCommand debugMove;
    debugMove.targetPositionRadians.assign(robotSpec.jointCount, 0.0);
    debugMove.targetPositionRadians[0] = glm::radians(30.0);
    debugMove.velocityScale = 1.0;
    debugMove.accelerationScale = 1.0;

    const grasplink::robotics::Result moveResult = m_RobotController->MoveJoint(debugMove);
    if (!moveResult)
    {
        std::cerr << moveResult.message << '\n';
        return false;
    }

    return true;
}

void ViewerApp::MainLoop()
{
    using Clock = std::chrono::steady_clock;
    auto lastFrameTime = Clock::now();

    while (!m_Window->ShouldClose())
    {
        const auto currentFrameTime = Clock::now();
        float dt = std::chrono::duration<float>(currentFrameTime - lastFrameTime).count();
        lastFrameTime = currentFrameTime;

        // Debugger 정지 등으로 매우 큰 frame dt가 들어와 한 번에 큰 각도 변화가 생기는 것을 제한한다.
        if (dt > 0.1F) dt = 0.1F;

        m_Window->PollEvents();
        m_CameraController->OnUpdate();

        int framebufferWidth = 0;
        int framebufferHeight = 0;
        m_Window->GetFramebufferSize(framebufferWidth, framebufferHeight);
        if (framebufferWidth <= 0 || framebufferHeight <= 0) continue;

        m_Renderer->Resize(framebufferWidth, framebufferHeight);
        m_Camera->SetAspectRatio(
            static_cast<float>(framebufferWidth) /
            static_cast<float>(framebufferHeight));

        /*
         * 현재 frame의 로봇 데이터 흐름:
         *
         * 1) Controller::Update(dt)
         *    target q를 향해 현재 q/dq를 시간에 따라 갱신.
         *
         * 2) GetState()
         *    갱신된 RobotState snapshot을 얻음.
         *
         * 3) RobotTransformAdapter::Apply()
         *    q[rad]를 GLB/Flecs Joint local rotation으로 변환.
         *
         * 4) 이후 Flecs TransformSystem이 parent-child hierarchy를 따라 world transform을 계산.
         *
         * 현재는 render frame dt를 그대로 제어 dt로 사용한다.
         * 향후 Fixed Control Loop에서는 1~3을 고정 주기 accumulator loop로 분리한다.
         */
        m_RobotController->Update(static_cast<double>(dt));
        m_RobotTransformAdapter->Apply(m_RobotController->GetState());

        m_Renderer->BeginFrame();
        m_SceneManager->OnUpdate(dt);
        m_World.progress(dt);
        m_Renderer->EndFrame();
        m_Window->SwapBuffers();
    }
}

void ViewerApp::Shutdown()
{
    /*
     * Adapter가 Controller state/robot Entity를 참조하므로 먼저 제거하고,
     * 그 다음 Controller를 Disconnect한 뒤 소유 객체를 파괴한다.
     */
    m_RobotTransformAdapter.reset();

    if (m_RobotController)
        m_RobotController->Disconnect();
    m_RobotController.reset();

    m_SceneManager.reset();
    m_AssetManager.reset();
    m_World.reset();

    m_CameraController.reset();
    m_Camera.reset();
    m_Renderer.reset();
    m_Window.reset();
}
