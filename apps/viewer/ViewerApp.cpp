#include "ViewerApp.h"

#include "Camera.h"
#include "Entity.h"
#include "Mesh.h"
#include "Renderable.h"
#include "Renderer.h"
#include "RenderSystem.h"
#include "Shader.h"
#include "Texture.h"
#include "Transform.h"
#include "Window.h"


#include <glm/glm.hpp>
#include <iostream>
#include <memory>

ViewerApp::ViewerApp() = default;
ViewerApp::~ViewerApp() = default;

int ViewerApp::Run()
{
    if (!Init()) {
        return -1;
    }

    MainLoop();
    Shutdown();
    return 0;
}

bool ViewerApp::Init()
{
    /*
        GPU resource를 생성하기 전에 Window가 OpenGL context를 만들어야 한다.
        따라서 Window -> Renderer -> flecs world -> Renderable Entity 순서로 초기화한다.
    */
    m_Window = std::make_unique<PoseLink::Window>(
        PoseLink::Window::Properties{1280, 720, "PoseLink Viewer", true});

    m_Renderer = std::make_unique<PoseLink::Renderer>();
    m_Renderer->Init();

    m_Camera =
        std::make_unique<PoseLink::Camera>(
            glm::vec3(2.0f, 2.0f, 3.0f),
            glm::vec3(0.0f, 0.0f, 0.0f),
            1280.0f / 720.0f
        );

    /*
        Scene/Object 관리의 단일 진입점.
        Renderer가 아니라 Application이 소유해야 entity 생성/제거와 OpenGL resource
        파괴 순서를 한 곳에서 결정할 수 있다.
    */
    m_World = std::make_unique<flecs::world>();
    // System
    m_RenderSystem = std::make_unique<PoseLink::RenderSystem>(*m_World);

    /*
        기존 Renderer가 직접 만들던 Cube resource를 Application 초기화 단계로 옮긴다.
        Mesh::CreateCube()는 unique_ptr을 반환하지만, 같은 Cube를 여러 Entity가 공유해야 하므로
        여기서 shared_ptr로 전환한다.
    */
    std::shared_ptr<PoseLink::Mesh> cubeMesh(PoseLink::Mesh::CreateCube());
    auto cubeShader = PoseLink::Shader::Create("shaders/Cube.glsl");
    cubeShader->Bind();
    cubeShader->SetInt("u_Texture", 0);
    cubeShader->UnBind();

    const unsigned char pixels[] = {
        255,   0,   0, 255,
          0, 255,   0, 255,
          0,   0, 255, 255,
        255, 255,   0, 255
    };
    auto cubeTexture = std::make_shared<PoseLink::Texture>(2, 2);
    cubeTexture->Update(pixels);

    /*
        Renderable 하나를 값으로 만들어 두 Entity에 복사한다.
        복사되는 것은 shared_ptr뿐이므로 Cube geometry, shader program, texture는 각각 하나만 생성된다.
    */
    const PoseLink::Renderable cubeRenderable{cubeMesh, cubeShader, cubeTexture};

    /*
        flecs entity는 이름을 Renderer가 해석하기 위해 사용하는 것이 아니다.
        디버깅과 식별을 위해 이름을 붙이고, 실제 렌더링 대상 여부는 component 조합으로 결정한다.
    */
    PoseLink::Transform cubeATransform;
    cubeATransform.position.x = -1.0f;
    m_World->entity("CubeA")
        .set<PoseLink::Transform>(cubeATransform)
        .set<PoseLink::Renderable>(cubeRenderable);

    PoseLink::Transform cubeBTransform;
    cubeBTransform.position.x = 1.0f;
    m_World->entity("CubeB")
        .set<PoseLink::Transform>(cubeBTransform)
        .set<PoseLink::Renderable>(cubeRenderable);


    return true;
}

void ViewerApp::MainLoop()
{
    while (!m_Window->ShouldClose()) {
        m_Window->PollEvents();
        m_Renderer->BeginFrame();
        // Renderer는 Transform + Renderable query로 두 Cube를 모두 찾는다.
        m_RenderSystem->Render(
            *m_Renderer,
            *m_Camera
        );
        m_Window->SwapBuffers();
    }
}

void ViewerApp::Shutdown()
{
    /*
        flecs world가 먼저 파괴되면 Renderable component의 shared_ptr가 GPU resource를 해제한다.
        그 다음 Renderer, 마지막으로 OpenGL context를 가진 Window를 파괴한다.
    */
    m_RenderSystem.reset();
    m_World.reset();
    m_Camera.reset();
    m_Renderer.reset();
    m_Window.reset();
}
