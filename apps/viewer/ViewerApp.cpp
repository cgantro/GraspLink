#include "ViewerApp.h"

#include "Camera.h"
#include "CadVisualRig.h"
#include "Grasping.h"
#include "Mesh.h"
#include "Renderable.h"
#include "RenderContext.h"
#include "Renderer.h"
#include "RenderSystem.h"
#include "RobotKinematics.h"
#include "Shader.h"
#include "Texture.h"
#include "Transform.h"
#include "Window.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <array>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
PoseLink::Transform ToTransform(const PoseLink::Kinematics::RigidTransform& pose)
{
    PoseLink::Transform transform{};
    transform.position = {pose.position.x, pose.position.y, pose.position.z};
    transform.rotation = glm::normalize(glm::quat(
        pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z));
    return transform;
}

std::string FindHcr12aGlb()
{
    const std::filesystem::path relative{"assets/hcr12a/HCR12A_R00.glb"};
    const std::array<std::filesystem::path, 2U> candidates{
        std::filesystem::current_path() / relative,
        std::filesystem::path{GRASPLINK_SOURCE_DIR} / relative};

    for (const std::filesystem::path& candidate : candidates)
    {
        if (std::filesystem::is_regular_file(candidate)) { return candidate.string(); }
    }

    throw std::runtime_error(
        "HCR-12A GLB is missing; run the CAD export or copy assets/hcr12a next to the simulator");
}
} // namespace

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
    m_Config = SimulatorConfig::Load();

    m_Window = std::make_unique<PoseLink::Window>(
        PoseLink::Window::Properties{1280, 720, "GraspLink HCR-12A Simulator", true});
    m_Renderer = std::make_unique<PoseLink::Renderer>();
    m_Renderer->Init();
    m_Camera = std::make_unique<PoseLink::Camera>(
        glm::vec3(2.3F, 1.7F, 2.8F), glm::vec3(0.35F, 0.45F, -0.55F), 1280.0F / 720.0F);

    m_World.set<PoseLink::RenderContext>({m_Renderer.get(), m_Camera.get()});
    m_World.import<PoseLink::RenderSystem>();

    const auto specification = PoseLink::Kinematics::RobotSpecification::MakeHcr12aNominal();
    m_ForwardKinematics = std::make_unique<PoseLink::Kinematics::ForwardKinematics>(specification);

    PoseLink::Kinematics::IkOptions ikOptions{};
    ikOptions.maximumIterations = m_Config.ikMaximumIterations;
    ikOptions.damping = m_Config.ikDamping;
    m_IkSolver = std::make_unique<PoseLink::Kinematics::DampedLeastSquaresIkSolver>(
        *m_ForwardKinematics, ikOptions);

    m_GraspPoseCalculator = std::make_unique<PoseLink::Kinematics::GraspPoseCalculator>(
        PoseLink::Kinematics::RigidTransform{});
    m_GraspController = std::make_unique<PoseLink::Kinematics::GraspController>(
        PoseLink::Kinematics::GraspThresholds{
            m_Config.graspPositionToleranceMetres,
            m_Config.graspOrientationToleranceRadians});

    const auto zeroFrames = m_ForwardKinematics->Evaluate(PoseLink::Kinematics::JointState{});
    m_CadVisualRig = std::make_unique<CadVisualRig>(zeroFrames);

    std::shared_ptr<PoseLink::Mesh> cubeMesh(PoseLink::Mesh::CreateCube());
    auto cubeShader = PoseLink::Shader::Create("shaders/Cube.glsl");
    cubeShader->Bind();
    cubeShader->SetInt("u_Texture", 0);
    cubeShader->UnBind();

    const unsigned char pixels[] = {
        235, 102, 35, 255, 45, 45, 45, 255,
        255, 159, 28, 255, 85, 85, 85, 255,
    };
    auto texture = std::make_shared<PoseLink::Texture>(2, 2);
    texture->Update(pixels);
    const PoseLink::Renderable cubeRenderable{cubeMesh, cubeShader, texture};

    const std::vector<PoseLink::StaticGlbMesh> hcrMeshes = PoseLink::Mesh::LoadStaticGlb(FindHcr12aGlb());
    const auto& expectedMeshNames = CadVisualRig::ExpectedMeshNames();
    if (hcrMeshes.size() != expectedMeshNames.size())
    {
        throw std::runtime_error("HCR-12A GLB must contain base plus six link meshes");
    }

    for (std::size_t index = 0; index < m_LinkEntities.size(); ++index)
    {
        if (hcrMeshes[index].name != expectedMeshNames[index] || !hcrMeshes[index].mesh)
        {
            throw std::runtime_error("HCR-12A GLB node name/order does not match the visual rig contract");
        }

        const std::string name = std::string(expectedMeshNames[index]);
        m_LinkEntities[index] = m_World.entity(name.c_str())
            .set<PoseLink::Transform>({})
            .set<PoseLink::Renderable>({hcrMeshes[index].mesh, cubeShader, texture});
    }

    m_TrackedEntity = m_World.entity("TargetObject")
        .set<PoseLink::Transform>({})
        .set<PoseLink::Renderable>(cubeRenderable);
    m_TcpEntity = m_World.entity("Hcr12aTcp")
        .set<PoseLink::Transform>({})
        .set<PoseLink::Renderable>(cubeRenderable);

    // Simulation-only baseline target. Future keyboard/GUI controls can update
    // this state directly without introducing a transport/protocol layer.
    m_ObjectPose.position = {0.65F, 0.45F, 0.0F};
    m_ObjectPose.orientation = m_Config.targetOrientation;
    m_DesiredTcp = m_GraspPoseCalculator->CalculateTcpTarget(m_ObjectPose);

    return true;
}

void ViewerApp::UpdateRobot(float deltaSeconds)
{
    const PoseLink::Kinematics::IkResult result = m_IkSolver->Solve(m_DesiredTcp, m_CurrentJoints);

    if (result.status == PoseLink::Kinematics::IkStatus::Converged)
    {
        constexpr float kMaximumJointSpeedRadiansPerSecond = 2.2F;
        const float maximumStep = kMaximumJointSpeedRadiansPerSecond * std::max(deltaSeconds, 0.0F);

        for (std::size_t i = 0; i < m_CurrentJoints.radians.size(); ++i)
        {
            const float difference = result.joints.radians[i] - m_CurrentJoints.radians[i];
            m_CurrentJoints.radians[i] += std::clamp(difference, -maximumStep, maximumStep);
        }
    }

    const auto frames = m_ForwardKinematics->Evaluate(m_CurrentJoints);

    if (m_GraspController->IsAttached())
    {
        m_GraspController->UpdateAttachedObject(frames.tcp, m_ObjectPose);
    }
    else if (result.status == PoseLink::Kinematics::IkStatus::Converged)
    {
        static_cast<void>(m_GraspController->TryAttach(frames.tcp, m_ObjectPose, m_DesiredTcp));
    }

    const auto visualTransforms = m_CadVisualRig->Evaluate(frames);
    for (std::size_t i = 0; i < m_LinkEntities.size(); ++i)
    {
        m_LinkEntities[i].set<PoseLink::Transform>(ToTransform(visualTransforms[i]));
    }

    PoseLink::Transform objectTransform = ToTransform(m_ObjectPose);
    objectTransform.scale = {0.10F, 0.10F, 0.10F};
    m_TrackedEntity.set<PoseLink::Transform>(objectTransform);

    PoseLink::Transform tcpTransform = ToTransform(frames.tcp);
    tcpTransform.scale = {0.055F, 0.055F, 0.055F};
    m_TcpEntity.set<PoseLink::Transform>(tcpTransform);
}

void ViewerApp::Update(float deltaSeconds)
{
    UpdateRobot(deltaSeconds);
}

void ViewerApp::MainLoop()
{
    m_LastFrameTime = static_cast<float>(glfwGetTime());

    while (!m_Window->ShouldClose())
    {
        const float currentTime = static_cast<float>(glfwGetTime());
        float deltaSeconds = currentTime - m_LastFrameTime;
        m_LastFrameTime = currentTime;
        deltaSeconds = std::clamp(deltaSeconds, 0.0F, 0.1F);

        m_Window->PollEvents();
        Update(deltaSeconds);
        m_Renderer->BeginFrame();
        m_World.progress(deltaSeconds);
        m_Window->SwapBuffers();
    }
}

void ViewerApp::Shutdown()
{
    m_GraspController.reset();
    m_GraspPoseCalculator.reset();
    m_IkSolver.reset();
    m_CadVisualRig.reset();
    m_ForwardKinematics.reset();

    m_TrackedEntity = flecs::entity::null();
    m_TcpEntity = flecs::entity::null();
    m_World.reset();

    m_Camera.reset();
    m_Renderer.reset();
    m_Window.reset();
}
