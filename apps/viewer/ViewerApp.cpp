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
constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 720;

const char* kWindowTitle = "GraspLink Viewer";

const glm::vec3 kCameraPosition{2.0F, 1.35F, 1.15F};
const glm::vec3 kCameraTarget{0.05F, 0.50F, 0.40F};

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
    // -------------------------------------------------------------------------
    // Window
    // -------------------------------------------------------------------------
    m_Window = std::make_unique<Window>(
        Window::Properties{kWindowWidth, kWindowHeight, kWindowTitle, true});

    // -------------------------------------------------------------------------
    // Renderer
    // -------------------------------------------------------------------------
    int framebufferWidth = 0;
    int framebufferHeight = 0;
    m_Window->GetFramebufferSize(framebufferWidth, framebufferHeight);

    m_Renderer = std::make_unique<Renderer>();
    m_Renderer->Init(framebufferWidth, framebufferHeight);

    // -------------------------------------------------------------------------
    // Camera
    // -------------------------------------------------------------------------
    m_Camera = std::make_unique<Camera>(
        kCameraPosition,
        kCameraTarget,
        static_cast<float>(kWindowWidth) / static_cast<float>(kWindowHeight));

    /*
        Camera는 View/Projection 계산만 담당하고,
        mouse 입력을 해석하는 책임은 OrbitCameraController에 둔다.
    */
    m_CameraController = std::make_unique<OrbitCameraController>(
        *m_Camera,
        *m_Window);

    // -------------------------------------------------------------------------
    // Flecs
    // -------------------------------------------------------------------------
    m_World.set<RenderContext>({m_Renderer.get(), m_Camera.get()});
    m_World.import<TransformSystemModule>();
    m_World.import<RenderSystemModule>();

    // -------------------------------------------------------------------------
    // Scene
    // -------------------------------------------------------------------------
    m_SceneManager = std::make_unique<SceneManager>(m_World);
    m_SceneManager->LoadScene<Scene>();
    m_SceneManager->OnUpdate(0.0F);

    Scene* scene = m_SceneManager->GetActiveScene();

    m_AssetManager = std::make_unique<AssetManager>();

    auto robotShader = Shader::Create("shaders/Robot.glsl");
    auto gridShader = Shader::Create("shaders/Grid.glsl");

    // -------------------------------------------------------------------------
    // HCR-12A GLB Loading Test
    // -------------------------------------------------------------------------
    ModelResource robotModel = GltfLoader::LoadGLB("HCR12A_R00.glb");
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

    // Robot Controller
    m_RobotJointController = std::make_unique<RobotJointController>(robotRoot);

    // J1 테스트
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

        // Debugger 정지 등으로 비정상적으로 큰 dt가 Simulation에 전달되는 것을 막는다.
        if (dt > 0.1F) dt = 0.1F;

        /*
            PollEvents가 GLFW callback을 실행하고 입력 상태를 최신화한 뒤
            Camera Controller가 mouse drag / wheel을 처리한다.
        */
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

        /*
            Frame 처리 순서:
                Input / Camera
                -> Scene Update
                -> Flecs Systems
                -> Transform
                -> Render
        */
        m_SceneManager->OnUpdate(dt);
        m_World.progress(dt);

        m_Renderer->EndFrame();
        m_Window->SwapBuffers();
    }
}

void ViewerApp::Shutdown()
{
    m_RobotJointController.reset();
    m_SceneManager.reset();
    m_AssetManager.reset();
    m_World.reset();

    // Controller가 Camera/Window reference를 가지므로 먼저 제거한다.
    m_CameraController.reset();
    m_Camera.reset();
    m_Renderer.reset();
    m_Window.reset();
}
