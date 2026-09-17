#include "ViewerApp.h"

#include "Camera.h"
#include "Material.h"
#include "Mesh.h"
#include "Renderable.h"
#include "RenderContext.h"
#include "Renderer.h"
#include "RenderSystem.h"
#include "Shader.h"
#include "Transform.h"
#include "Window.h"

#include <GLFW/glfw3.h>

#include <memory>
#include <stdexcept>

namespace
{
constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 720;
const char* kWindowTitle = "GraspLink Viewer";
const glm::vec3 kCameraPosition{2.0F, 1.35F, 2.15F};
const glm::vec3 kCameraTarget{0.05F, 0.50F, 0.40F};
const glm::vec3 kDebugPosition{0.0F, 0.5F, 0.0F};
const glm::vec3 kDebugScale{0.25F, 0.25F, 0.25F};
}

ViewerApp::ViewerApp() = default;
ViewerApp::~ViewerApp() = default;

int ViewerApp::Run()
{
    if (!Init()) { return -1; }
    MainLoop();
    Shutdown();
    return 0;
}

bool ViewerApp::Init()
{
    window_ = std::make_unique<PoseLink::Window>(PoseLink::Window::Properties{
        kWindowWidth, kWindowHeight, kWindowTitle, true});
    renderer_ = std::make_unique<PoseLink::Renderer>();
    renderer_->Init();
    camera_ = std::make_unique<PoseLink::Camera>(
        kCameraPosition,
        kCameraTarget,
        static_cast<float>(kWindowWidth) / static_cast<float>(kWindowHeight));

    world_.set<PoseLink::RenderContext>({renderer_.get(), camera_.get()});
    world_.import<PoseLink::RenderSystem>();

    auto shader = PoseLink::Shader::Create("shaders/Debug.glsl");
    auto material = std::make_shared<PoseLink::Material>(
        glm::vec4(0.18F, 0.45F, 0.85F, 1.0F), 0.0F, 0.8F);
    std::shared_ptr<PoseLink::Mesh> mesh(PoseLink::Mesh::CreateCube());
    debugEntity_ = world_.entity("DebugCube")
        .set<PoseLink::Transform>(PoseLink::Transform{
            kDebugPosition, glm::quat(1.0F, 0.0F, 0.0F, 0.0F), kDebugScale})
        .set<PoseLink::Renderable>({mesh, shader, material});
    return true;
}

void ViewerApp::MainLoop()
{
    while (!window_->ShouldClose())
    {
        window_->PollEvents();
        renderer_->BeginFrame();
        world_.progress(0.0F);
        window_->SwapBuffers();
    }
}

void ViewerApp::Shutdown()
{
    debugEntity_ = flecs::entity::null();
    world_.reset();
    camera_.reset();
    renderer_.reset();
    window_.reset();
}
