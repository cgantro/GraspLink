#include "ViewerApp.h"

#include "Camera.h"
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

const glm::vec3 kCameraPosition{2.0F,1.35F,1.15F};

const glm::vec3 kCameraTarget{0.05F,0.50F,0.40F};

const glm::vec3 kDebugPosition{0.0F,0.5F,0.0F};

const glm::vec3 kDebugScale{0.25F};


} // namespace


ViewerApp::ViewerApp() = default;

ViewerApp::~ViewerApp(){
    Shutdown();
}


int ViewerApp::Run()
{
    if (!Init())
    {
        return -1;
    }

    MainLoop();

    return 0;
}


bool ViewerApp::Init()
{
    // -------------------------------------------------------------------------
    // Window
    // -------------------------------------------------------------------------

    m_Window =std::make_unique<Window>(Window::Properties{kWindowWidth,kWindowHeight,kWindowTitle,true});


    // -------------------------------------------------------------------------
    // Renderer
    // -------------------------------------------------------------------------

    int framebufferWidth = 0;
    int framebufferHeight = 0;

    m_Window->GetFramebufferSize(framebufferWidth,framebufferHeight);

    m_Renderer = std::make_unique<Renderer>();
    m_Renderer->Init(framebufferWidth,framebufferHeight);


    // -------------------------------------------------------------------------
    // Camera
    // -------------------------------------------------------------------------

    m_Camera =std::make_unique<Camera>(kCameraPosition, kCameraTarget, 
                                    static_cast<float>(kWindowWidth) / static_cast<float>(kWindowHeight));


    // -------------------------------------------------------------------------
    // Flecs
    // -------------------------------------------------------------------------

    m_World.set<RenderContext>({m_Renderer.get(),m_Camera.get()});

    m_World.import<TransformSystemModule>();
    m_World.import<RenderSystemModule>();


    // -------------------------------------------------------------------------
    // Scene
    // -------------------------------------------------------------------------

    /*
        아직 SimulationScene 같은 구체 Scene을 만들지 않았으므로
        기본 Scene을 사용한다.
    */

    m_SceneManager =
        std::make_unique<SceneManager>(
            m_World);

    m_SceneManager->LoadScene<Scene>();

    /*
        초기 Scene 전환 등을 처리한다.
    */
    m_SceneManager->OnUpdate(0.0F);

    Scene* scene = m_SceneManager->GetActiveScene();

    // Asset Manager
    m_AssetManager = std::make_unique<AssetManager>();
    
    auto robotShader = Shader::Create("shaders/Robot.glsl");
    auto gridShader =Shader::Create("shaders/Grid.glsl");
    // -------------------------------------------------------------------------
    // HCR-12A GLB Loading Test
    // -------------------------------------------------------------------------
    ModelResource robotModel =GltfLoader::LoadGLB("HCR12A_R00.glb");

    // CPU MeshData → GPU Mesh / Material
    m_AssetManager->UploadModel(robotModel);

    // ModelResource → Flecs Entity hierarchy
    Entity robotRoot = PrefabFactory::CreateModel(*scene,robotModel,*m_AssetManager,robotShader);

    ModelResource planeModel = GltfLoader::LoadGLB("plane.glb");
    m_AssetManager->UploadModel(planeModel);
    Entity planeRoot = PrefabFactory::CreateModel(*scene,planeModel,*m_AssetManager,gridShader);

    // Robot Controller
    m_RobotJointController = std::make_unique<RobotJointController>(robotRoot);

    // J1 테스트
    m_RobotJointController->SetJointPosition(0,glm::radians(30.0F),glm::vec3{0.0F,1.0F,0.0F});
    
    return true;
}


void ViewerApp::MainLoop()
{
    using Clock =std::chrono::steady_clock;

    auto lastFrameTime = Clock::now();


    while (!m_Window->ShouldClose())
    {
        const auto currentFrameTime =Clock::now();

        /*
            현재 frame과 이전 frame 사이의 실제 경과 시간.

            duration<float> 단위:
                seconds
        */
        float dt =std::chrono::duration<float>(currentFrameTime -lastFrameTime).count();

        lastFrameTime =currentFrameTime;

        /*
            디버거 정지 등으로 인해
            갑자기 매우 큰 dt가 전달되는 것을 방지한다.
        */
        if (dt > 0.1F)
        {
            dt = 0.1F;
        }
        m_Window->PollEvents();
        
        int framebufferWidth = 0;
        int framebufferHeight = 0;
        m_Window->GetFramebufferSize(framebufferWidth,framebufferHeight);
        if (framebufferWidth <= 0 || framebufferHeight <= 0) continue;

        m_Renderer->Resize(framebufferWidth,framebufferHeight);
        m_Camera->SetAspectRatio(static_cast<float>(framebufferWidth)/static_cast<float>(framebufferHeight));

        m_Renderer->BeginFrame();
        /*
            Frame 처리 순서:

            SceneManager
                ↓
            Scene Update
                ↓
            Flecs Systems
                ↓
            Transform
                ↓
            Render
        */

        m_SceneManager->OnUpdate(dt);
        m_World.progress(dt);
        m_Renderer->EndFrame();
        m_Window->SwapBuffers();
    }
}


void ViewerApp::Shutdown()
{
    m_SceneManager.reset();
    m_AssetManager.reset();
    m_World.reset();

    m_Camera.reset();
    m_Renderer.reset();
    m_Window.reset();
}

