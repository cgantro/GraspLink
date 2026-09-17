#pragma once

#include "KinematicsTypes.h"
#include "SimulatorConfig.h"

#include <array>
#include <cstdint>
#include <flecs.h>
#include <memory>

namespace PoseLink
{
class Window;
class Renderer;
class Camera;
class SerialTransport;
class LatestSerialTargetReceiver;
namespace Kinematics
{
class ForwardKinematics;
class DampedLeastSquaresIkSolver;
class GraspPoseCalculator;
class GraspController;
}
} // namespace PoseLink
class CadVisualRig;

/**
 * @brief UI/window lifetime와 renderer-independent robot domain을 결합하는 application root.
 *
 * ViewerApp은 serial transport, IK, Flecs 각각의 구현 세부를 다른 module에 위임한다. 여기의 책임은
 * 한 frame에서 `serial target -> desired TCP -> IK/FK -> CAD visual rig -> scene transform -> RobotState`
 * 순서를 유지하는 것이다. GPU resource가 든 Flecs world는 OpenGL context보다 먼저
 * 파괴해야 하므로 Shutdown()의 destruction order는 명시적으로 유지한다.
 */
class ViewerApp
{
public:
    ViewerApp();
    ~ViewerApp();
    int Run();

private:
    bool Init();
    void MainLoop();
    void Update(float deltaSeconds);
    void ApplyRemoteTarget();
    void UpdateRobot(float deltaSeconds);
    void PublishRobotState(PoseLink::GraspStatus status);
    void Shutdown();

    std::unique_ptr<PoseLink::Window> m_Window;
    std::unique_ptr<PoseLink::Renderer> m_Renderer;
    std::unique_ptr<PoseLink::Camera> m_Camera;
    flecs::world m_World;

    SimulatorConfig m_Config{};
    std::unique_ptr<PoseLink::SerialTransport> m_SerialTransport;
    std::unique_ptr<PoseLink::LatestSerialTargetReceiver> m_TargetReceiver;
    std::unique_ptr<PoseLink::Kinematics::ForwardKinematics> m_ForwardKinematics;
    std::unique_ptr<PoseLink::Kinematics::DampedLeastSquaresIkSolver> m_IkSolver;
    std::unique_ptr<PoseLink::Kinematics::GraspPoseCalculator> m_GraspPoseCalculator;
    std::unique_ptr<PoseLink::Kinematics::GraspController> m_GraspController;
    std::unique_ptr<CadVisualRig> m_CadVisualRig;

    PoseLink::Kinematics::JointState m_CurrentJoints{};
    PoseLink::Kinematics::RigidTransform m_ObjectPose{};
    PoseLink::Kinematics::RigidTransform m_DesiredTcp{};
    std::array<flecs::entity, 7> m_LinkEntities{};
    flecs::entity m_TrackedEntity{flecs::entity::null()};
    flecs::entity m_TcpEntity{flecs::entity::null()};
    std::uint32_t m_StateSequence = 0U;
    std::uint32_t m_AcknowledgedTargetSequence = 0U;
    // State packet is semantic feedback, not a per-frame telemetry stream.
    PoseLink::GraspStatus m_LastPublishedStatus = PoseLink::GraspStatus::ProtocolError;
    float m_LastFrameTime = 0.0F;
};
