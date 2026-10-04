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

#include "robot/RobotJointController.h"

#include "assets/GltfLoader.h"
#include "assets/AssetManager.h"
#include "assets/PrefabFactory.h"

#include "components/RenderComponents.h"
#include "components/TransformComponents.h"

#include "scene/Scene.h"
#include "scene/SceneManager.h"

#include <chrono>
#include <iostream>
#include <memory>

namespace
{
// 실행 환경 기본값. UI/설정 파일이 생기기 전까지 ViewerApp의 bootstrap 값으로 사용한다.
constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 720;
const char* kWindowTitle = "GraspLink Viewer";

const glm::vec3 kCameraPosition{2.0F, 1.35F, 1.15F};
const glm::vec3 kCameraTarget{0.05F, 0.50F, 0.40F};

// TODO(FUTURE): graphics/simulation settings가 생기면 window/camera 기본값을 설정 객체로 이동한다.
const glm::vec3 kDebugPosition{0.0F, 0.5F, 0.0F};
const glm::vec3 kDebugScale{0.25F};

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
    // 1) Window가 OpenGL Context를 만든다. Renderer보다 먼저 생성되어야 GL 호출이 유효하다.
    m_Window = std::make_unique<Window>(
        Window::Properties{kWindowWidth, kWindowHeight, kWindowTitle, true});

    // 2) Renderer는 이미 생성된 Context 위에 framebuffer/shadow resource를 만든다.
    int framebufferWidth = 0;
    int framebufferHeight = 0;
    m_Window->GetFramebufferSize(framebufferWidth, framebufferHeight);

    m_Renderer = std::make_unique<Renderer>();
    m_Renderer->Init(framebufferWidth, framebufferHeight);

    // 3) Camera는 수학 상태만 보관하고 mouse 입력 해석은 Controller에 분리한다.
    m_Camera = std::make_unique<Camera>(
        kCameraPosition,
        kCameraTarget,
        static_cast<float>(kWindowWidth) / static_cast<float>(kWindowHeight));

    m_CameraController = std::make_unique<OrbitCameraController>(
        *m_Camera,
        *m_Window);

    // 4) Flecs World에 System들이 공통으로 사용할 Renderer/Camera context를 등록한다.
    m_World.set<RenderContext>({m_Renderer.get(), m_Camera.get()});
    m_World.import<TransformSystemModule>();
    m_World.import<RenderSystemModule>();

    // TODO(FUTURE): Robot/Plane 생성은 SimulationScene::OnEnter()로 이동한다.
    m_SceneManager = std::make_unique<SceneManager>(m_World);
    m_SceneManager->LoadScene<Scene>();
    m_SceneManager->OnUpdate(0.0F);

    Scene* scene = m_SceneManager->GetActiveScene();

    // 5) Loader는 CPU ModelResource를 만들고 AssetManager는 GPU resource로 업로드한다.
    m_AssetManager = std::make_unique<AssetManager>();

    auto robotShader = Shader::Create("shaders/Robot.glsl");
    auto gridShader = Shader::Create("shaders/Grid.glsl");

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

    // 6) Controller는 GLB hierarchy에서 J1~J6를 찾아 bind rotation을 보존한다.
    m_RobotJointController = std::make_unique<RobotJointController>(robotRoot);

    // TODO(FUTURE): 임시 J1 축 검증 코드. J1~J6 axis/limit 확정 후 RobotModel 초기 상태로 교체한다.
    // 현재는 HCR-12A 공식 angle limit/velocity limit이 아직 Controller에 적용되지 않는다.
    m_RobotJointController->SetJointPosition(
        0,
        glm::radians(30.0F),
        glm::vec3{0.0F, 1.0F, 0.0F});

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

        // Debugger breakpoint 등으로 큰 dt가 들어오면 simulation이 한 frame에 크게 튈 수 있다.
        if (dt > 0.1F) dt = 0.1F;

        // GLFW event를 먼저 poll해야 scroll callback과 mouse button 상태가 최신 값이 된다.
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

        m_Renderer->BeginFrame();

        // Scene logic -> ECS systems(Transform/Render) 순으로 진행한다.
        m_SceneManager->OnUpdate(dt);
        m_World.progress(dt);

        // MSAA framebuffer를 default framebuffer로 resolve한 뒤 화면에 표시한다.
        m_Renderer->EndFrame();
        m_Window->SwapBuffers();
    }
}

void ViewerApp::Shutdown()
{
    // 참조를 가진 상위 controller/scene부터 제거하고 마지막에 Window/Context를 파괴한다.
    m_RobotJointController.reset();
    m_SceneManager.reset();
    m_AssetManager.reset();
    m_World.reset();

    m_CameraController.reset();
    m_Camera.reset();
    m_Renderer.reset();
    m_Window.reset();
}
