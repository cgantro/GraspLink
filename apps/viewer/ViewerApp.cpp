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

#include <memory>

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

    auto shader = Shader::Create("shaders/Debug.glsl");
    auto material = std::make_shared<Material>(
        glm::vec4(0.18F, 0.45F, 0.85F, 1.0F), 0.0F, 0.8F);
    std::shared_ptr<Mesh> mesh = Mesh::CreateCube();

    // DebugCube도 실제 씬 엔티티와 같은 Local/World pair를 사용한다.
    auto debugEntity = m_World.entity("DebugCube");
    debugEntity
        .set<Position, Local>(Position{kDebugPosition})
        .set<Rotation, Local>(Rotation{glm::vec3(0.0F)})
        .set<Scale, Local>(Scale{kDebugScale})
        .set<TransformMatrix, Local>(TransformMatrix{})
        .set<TransformMatrix, World>(TransformMatrix{})
        .set<MeshFilter>(MeshFilter{mesh})
        .set<MeshRenderer>(MeshRenderer{shader, material, true});

    return true;
}

void ViewerApp::MainLoop()
{
    while (!m_Window->ShouldClose())
    {
        m_Window->PollEvents();
        m_Renderer->BeginFrame();
        m_World.progress(0.0F);
        m_Window->SwapBuffers();
    }
}

void ViewerApp::Shutdown()
{
    m_World.reset();
    m_Camera.reset();
    m_Renderer.reset();
    m_Window.reset();
}
