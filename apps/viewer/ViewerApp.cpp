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

    const auto& robotSpec = grasplink::robotics::models::hanwha::kHcr12a;

    auto simController =
        std::make_unique<grasplink::robotics::backends::simulation::SimRobotController>(robotSpec);
    const grasplink::robotics::Result connectResult = simController->Connect();
    if (!connectResult)
    {
        std::cerr << connectResult.message << '\n';
        return false;
    }

    m_RobotController = std::move(simController);
    m_RobotTransformAdapter =
        std::make_unique<grasplink::viewer::robotics::RobotTransformAdapter>(robotRoot, robotSpec);

    // 임시 J1 30도 검증 동작. 향후 입력/UI/trajectory 계층으로 교체한다.
    grasplink::robotics::JointMoveCommand debugMove;
    debugMove.targetPositionRadians.assign(robotSpec.jointCount, 0.0);
    debugMove.targetPositionRadians[0] = glm::radians(30.0);
    debugMove.velocityScale = 0.06;
    debugMove.accelerationScale = 0.06;

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
