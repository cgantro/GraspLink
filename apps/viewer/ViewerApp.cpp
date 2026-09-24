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
#include "components/RenderComponents.h"
#include "components/TransformComponents.h"
#include "scene/Scene.h"
#include "scene/SceneManager.h"
#include <memory>
#include <chrono>

namespace
{
constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 720;
const char* kWindowTitle = "GraspLink Viewer";
const glm::vec3 kCameraPosition{2.0F, 1.35F, 2.15F};
const glm::vec3 kCameraTarget{0.05F, 0.50F, 0.40F};
const glm::vec3 kDebugPosition{0.0F, 0.5F, 0.0F};
const glm::vec3 kDebugScale{0.25F};
}

ViewerApp::ViewerApp() = default;
ViewerApp::~ViewerApp() = default;

int ViewerApp::Run()
{
    if (!Init())
    {
        return -1;
    }

    MainLoop();
    Shutdown();
    return 0;
}

bool ViewerApp::Init()
{
    m_Window = std::make_unique<Window>(Window::Properties{
        kWindowWidth, kWindowHeight, kWindowTitle, true});
    m_Renderer = std::make_unique<Renderer>();
    m_Renderer->Init();
    m_Camera = std::make_unique<Camera>(
        kCameraPosition,
        kCameraTarget,
        static_cast<float>(kWindowWidth) /
            static_cast<float>(kWindowHeight));

    m_World.set<RenderContext>({m_Renderer.get(), m_Camera.get()});

    m_World.import<TransformSystemModule>();
    m_World.import<RenderSystemModule>();


     /*
        아직 SimulationScene 같은 구체 Scene을 만들지 않기로 했으므로
        현재는 기본 Scene을 그대로 사용한다.

        Scene::OnEnter() 등이 virtual이지만 pure virtual이 아니므로
        Scene 자체를 생성할 수 있다.
    */
    m_SceneManager = std::make_unique<SceneManager>(m_World);
    m_SceneManager->LoadScene<Scene>();
    m_SceneManager->OnUpdate(0.0f);

    Scene* scene = m_SceneManager->GetActiveScene();
    auto shader = Shader::Create("shaders/Debug.glsl");
    auto material = std::make_shared<Material>(
        glm::vec4(0.18F, 0.45F, 0.85F, 1.0F), 0.0F, 0.8F);
    std::shared_ptr<Mesh> mesh = Mesh::CreateCube();
    // DebugCube도 실제 씬 엔티티와 같은 Local/World pair를 사용한다.
    Entity debugCube = scene->CreateEntity("DebugCube");
    debugCube.SetLocalPosition(kDebugPosition);
    debugCube.SetLocalScale(kDebugScale);
    debugCube.Set<MeshFilter>(MeshFilter{mesh})
            .Set<MeshRenderer>(MeshRenderer{shader,material,true});
    return true;
}

void ViewerApp::MainLoop()
{   
    using Clock=std::chrono::steady_clock;
    auto lastFrameTime = Clock::now();

    while (!m_Window->ShouldClose())
    {   
        const auto currentFrameTime = Clock::now();

        /*
            현재 frame과 이전 frame 사이의 실제 경과 시간.
            duration<float>의 단위는 seconds.
        */
        float dt = std::chrono::duration<float>(currentFrameTime - lastFrameTime).count();
        lastFrameTime = currentFrameTime;
        if(dt > 0.1F) dt = 0.1F; // dt가 너무 길어진 경우, 갑자기 큰 dt가 전송되는걸 방지한다.
        m_Window->PollEvents();
        m_Renderer->BeginFrame();

        /*
            순서가 중요
            1. SceneManager
                - Scene 전환
                - update
            2. World
                - Transform
                - Render
            즉, Scene에서 Transform 변경 후, 같은 Frame의 ECS System이 그 값을 처리
        */
        m_SceneManager->OnUpdate(dt);
        m_World.progress(dt);
        m_Window->SwapBuffers();
    }
}

void ViewerApp::Shutdown()
{   
    m_SceneManager.reset();
    m_World.reset();
    m_Camera.reset();
    m_Renderer.reset();
    m_Window.reset();
}
