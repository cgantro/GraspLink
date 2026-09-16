#include "ViewerApp.h"

#include "Camera.h"
#include "Mesh.h"
#include "RenderContext.h"
#include "Renderable.h"
#include "Renderer.h"
#include "RenderSystem.h"
#include "Shader.h"
#include "Texture.h"
#include "Transform.h"
#include "Window.h"
#include "IPoseSource.h"
#include "SyntheticPoseSource.h"
#include "Pose.h"

#include <GLFW/glfw3.h>

namespace 
{
    PoseLink::Transform ToTransform(
    const PoseLink::Pose& pose){
    PoseLink::Transform transform;

    transform.position = {
        pose.position.x,
        pose.position.y,
        pose.position.z
    };

    /*
        GLM quaternion 생성자 순서:

        glm::quat(w, x, y, z)
    */
    transform.rotation =
        glm::normalize(
            glm::quat(
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z
            )
        );

    return transform;
}
} // namespace 

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

    m_PoseSource = std::make_unique<PoseLink::SyntheticPoseSource>();

    m_World.set<PoseLink::RenderContext>({
        m_Renderer.get(),
        m_Camera.get()
    });

    m_World.import<PoseLink::RenderSystem>();
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
    cubeATransform.position.x = 0.0f;
    m_TrackedEntity =
    m_World.entity("CubeA")
        .set<PoseLink::Transform>(
            cubeATransform
        )
        .set<PoseLink::Renderable>(
            cubeRenderable
        );

   

    PoseLink::Transform cubeBTransform;
    cubeBTransform.position.x = 1.5f;
    m_World.entity("CubeB")
    .set<PoseLink::Transform>(cubeBTransform)
    .set<PoseLink::Renderable>(cubeRenderable);


    return true;
}

void ViewerApp::Update(double elapsedSeconds)
{
    if (!m_PoseSource ||
        !m_TrackedEntity.is_alive())
    {
        return;
    }

    const std::optional<PoseLink::Pose> pose =
        m_PoseSource->Sample(elapsedSeconds);

    if (!pose) {
        return;
    }

    /*
        PoseSource가 만든 domain Pose를
        Viewer에서 사용하는 Transform으로 변환한다.

        RenderSystem이나 Renderer는
        PoseSource의 존재를 알 필요가 없다.
    */
    m_TrackedEntity.set<PoseLink::Transform>(
        ToTransform(*pose)
    );
}

void ViewerApp::MainLoop()
{
    m_LastFrameTime =
        static_cast<float>(glfwGetTime());

    while (!m_Window->ShouldClose())
    {
        const float currentTime =
            static_cast<float>(glfwGetTime());

        float dt = static_cast<float>(currentTime - m_LastFrameTime);

        m_LastFrameTime = currentTime;

        if (dt > 0.1f) {
            dt = 0.1f;
        }
        const double elapsedSeconds = currentTime - m_StartTime;
        m_Window->PollEvents();


        // 나중에 SyntheticPoseSource 등의
        // application-side update가 들어갈 위치
        Update(elapsedSeconds);

        m_Renderer->BeginFrame();

        // 등록된 flecs system 실행
        m_World.progress(dt);

        m_Window->SwapBuffers();
    }
}

void ViewerApp::Shutdown()
{
    /*
        flecs world가 먼저 파괴되면 Renderable component의 shared_ptr가 GPU resource를 해제한다.
        그 다음 Renderer, 마지막으로 OpenGL context를 가진 Window를 파괴한다.
    */
    m_TrackedEntity = flecs::entity::null();

    m_PoseSource.reset();
    m_World.reset();
    m_Camera.reset();
    m_Renderer.reset();
    m_Window.reset();
}
