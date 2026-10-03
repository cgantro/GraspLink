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

#include "assets/GltfLoader.h"

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

const glm::vec3 kCameraPosition{
    2.0F,
    1.35F,
    2.15F
};

const glm::vec3 kCameraTarget{
    0.05F,
    0.50F,
    0.40F
};

const glm::vec3 kDebugPosition{
    0.0F,
    0.5F,
    0.0F
};

const glm::vec3 kDebugScale{
    0.25F
};


void PrintNodeHierarchy(
    const ModelResource& model,
    int nodeIndex,
    int depth = 0)
{
    if (nodeIndex < 0 ||
        nodeIndex >= static_cast<int>(model.nodes.size()))
    {
        return;
    }

    const NodeData& node =
        model.nodes[nodeIndex];

    for (int i = 0; i < depth; ++i)
    {
        std::cout << "  ";
    }

    std::cout
        << node.name
        << " ["
        << nodeIndex
        << "]";

    if (node.meshIndex >= 0)
    {
        std::cout
            << " mesh="
            << node.meshIndex;
    }

    std::cout << '\n';

    for (const int childIndex :
         node.childrenIndices)
    {
        PrintNodeHierarchy(
            model,
            childIndex,
            depth + 1);
    }
}

} // namespace


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
    // -------------------------------------------------------------------------
    // Window
    // -------------------------------------------------------------------------

    m_Window =
        std::make_unique<Window>(
            Window::Properties{
                kWindowWidth,
                kWindowHeight,
                kWindowTitle,
                true
            });


    // -------------------------------------------------------------------------
    // Renderer
    // -------------------------------------------------------------------------

    m_Renderer =
        std::make_unique<Renderer>();

    m_Renderer->Init();


    // -------------------------------------------------------------------------
    // Camera
    // -------------------------------------------------------------------------

    m_Camera =
        std::make_unique<Camera>(
            kCameraPosition,
            kCameraTarget,
            static_cast<float>(kWindowWidth) /
                static_cast<float>(kWindowHeight));


    // -------------------------------------------------------------------------
    // Flecs
    // -------------------------------------------------------------------------

    m_World.set<RenderContext>({
        m_Renderer.get(),
        m_Camera.get()
    });

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

    Scene* scene =
        m_SceneManager->GetActiveScene();


    // -------------------------------------------------------------------------
    // Debug Cube
    // -------------------------------------------------------------------------

    auto shader =
        Shader::Create(
            "shaders/Debug.glsl");

    auto material =
        std::make_shared<Material>(
            glm::vec4(
                0.18F,
                0.45F,
                0.85F,
                1.0F),
            0.0F,
            0.8F);

    std::shared_ptr<Mesh> mesh =
        Mesh::CreateCube();

    Entity debugCube =
        scene->CreateEntity(
            "DebugCube");

    debugCube.SetLocalPosition(
        kDebugPosition);

    debugCube.SetLocalScale(
        kDebugScale);

    debugCube
        .Set<MeshFilter>(
            MeshFilter{
                mesh
            })
        .Set<MeshRenderer>(
            MeshRenderer{
                shader,
                material,
                true
            });


    // -------------------------------------------------------------------------
    // HCR-12A GLB Loading Test
    // -------------------------------------------------------------------------

    const ModelResource robotModel =
        GltfLoader::LoadGLB(
            "HCR12A_R00.glb");


    // -------------------------------------------------------------------------
    // Model Statistics
    // -------------------------------------------------------------------------

    std::size_t primitiveCount = 0;
    std::size_t vertexCount = 0;
    std::size_t indexCount = 0;

    for (const MeshData& meshData :
         robotModel.meshes)
    {
        primitiveCount +=
            meshData.subMeshes.size();

        vertexCount +=
            meshData.vertices.size();

        indexCount +=
            meshData.indices.size();
    }


    std::cout
        << "\n===== HCR-12A GLB =====\n"

        << "Nodes      : "
        << robotModel.nodes.size()
        << '\n'

        << "Meshes     : "
        << robotModel.meshes.size()
        << '\n'

        << "Materials  : "
        << robotModel.materials.size()
        << '\n'

        << "Primitives : "
        << primitiveCount
        << '\n'

        << "Vertices   : "
        << vertexCount
        << '\n'

        << "Indices    : "
        << indexCount
        << '\n'

        << "Root Index : "
        << robotModel.rootNodeIndex
        << '\n';


    // -------------------------------------------------------------------------
    // J1 ~ J6 Check
    // -------------------------------------------------------------------------

    std::cout
        << "\n===== Robot Joints =====\n";

    for (std::size_t i = 0;
         i < robotModel.nodes.size();
         ++i)
    {
        const NodeData& node =
            robotModel.nodes[i];

        if (node.name == "J1" ||
            node.name == "J2" ||
            node.name == "J3" ||
            node.name == "J4" ||
            node.name == "J5" ||
            node.name == "J6")
        {
            std::cout
                << node.name

                << " | index = "
                << i

                << " | parent = "
                << node.parentIndex

                << " | mesh = "
                << node.meshIndex

                << '\n';
        }
    }


    // -------------------------------------------------------------------------
    // Hierarchy
    // -------------------------------------------------------------------------

    std::cout
        << "\n===== Node Hierarchy =====\n";

    PrintNodeHierarchy(
        robotModel,
        robotModel.rootNodeIndex);


    return true;
}


void ViewerApp::MainLoop()
{
    using Clock =
        std::chrono::steady_clock;

    auto lastFrameTime =
        Clock::now();


    while (!m_Window->ShouldClose())
    {
        const auto currentFrameTime =
            Clock::now();

        /*
            현재 frame과 이전 frame 사이의 실제 경과 시간.

            duration<float> 단위:
                seconds
        */
        float dt =
            std::chrono::duration<float>(
                currentFrameTime -
                lastFrameTime)
                .count();

        lastFrameTime =
            currentFrameTime;


        /*
            디버거 정지 등으로 인해
            갑자기 매우 큰 dt가 전달되는 것을 방지한다.
        */
        if (dt > 0.1F)
        {
            dt = 0.1F;
        }


        m_Window->PollEvents();

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