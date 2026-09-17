#pragma once

#include "KinematicsTypes.h"
#include "SimulatorConfig.h"

#include <array>
#include <flecs.h>
#include <memory>

namespace PoseLink
{
class Window;
class Renderer;
class Camera;
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
 * @brief OpenGL/Flecs scene와 renderer-independent robot domain을 결합하는 application root.
 *
 * 한 frame에서 `simulation target -> desired TCP -> IK -> joint update -> FK -> CAD visual rig
 * -> scene transform -> grasp/attach` 순서를 유지한다. 외부 controller, protocol, serial/network
 * transport는 application 경계에 존재하지 않는다.
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
    void UpdateRobot(float deltaSeconds);
    void Shutdown();

    std::unique_ptr<PoseLink::Window> m_Window;
    std::unique_ptr<PoseLink::Renderer> m_Renderer;
    std::unique_ptr<PoseLink::Camera> m_Camera;
    flecs::world m_World;

    SimulatorConfig m_Config{};
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
    float m_LastFrameTime = 0.0F;
};
