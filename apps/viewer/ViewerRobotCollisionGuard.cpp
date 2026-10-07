#include "ViewerRobotCollisionGuard.h"

#include "Entity.h"
#include "TransformSystemModule.h"

#include "PhysicsWorld.h"
#include "simulation/components/PhysicsComponents.h"
#include "simulation/systems/PhysicsSystemModule.h"

#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/core/IGripperController.h"
#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/kinematics/GripperKinematics.h"

#include "viewer/robotics/RobotTransformAdapter.h"
#include "viewer/robotics/GripperTransformAdapter.h"
#include "simulation/robotics/RobotPhysicsAdapter.h"

#include <algorithm>
#include <cstring>
#include <glm/gtc/quaternion.hpp>

namespace grasplink::viewer
{
ViewerRobotCollisionGuard::ViewerRobotCollisionGuard(
    grasplink::robotics::backends::simulation::SimRobotController& robotController,
    grasplink::robotics::IGripperController& gripperController,
    grasplink::robotics::kinematics::RobotKinematics& robotKinematics,
    grasplink::viewer::robotics::RobotTransformAdapter& robotTransformAdapter,
    grasplink::simulation::RobotPhysicsAdapter& robotPhysicsAdapter,
    grasplink::robotics::kinematics::GripperKinematics& gripperKinematics,
    grasplink::viewer::robotics::GripperTransformAdapter& gripperTransformAdapter,
    flecs::world& world,
    grasplink::physics::PhysicsWorld& physicsWorld,
    grasplink::simulation::PhysicsSystemModule& physicsSystem,
    const Entity& robotRoot)
    : m_RobotController(robotController),
      m_GripperController(gripperController),
      m_RobotKinematics(robotKinematics),
      m_RobotTransformAdapter(robotTransformAdapter),
      m_RobotPhysicsAdapter(robotPhysicsAdapter),
      m_GripperKinematics(gripperKinematics),
      m_GripperTransformAdapter(gripperTransformAdapter),
      m_World(world),
      m_PhysicsWorld(physicsWorld),
      m_CandidateState(robotController.GetStateView()),
      m_SafeJointPositions(robotController.GetStateView().jointPositionRadians)
{
    std::vector<Entity> pending{robotRoot};
    while (!pending.empty())
    {
        Entity entity = pending.back();
        pending.pop_back();
        if (entity.Has<RigidBody>())
        {
            const auto layer = entity.Get<RigidBody>().collisionLayer;
            const char* entityName = entity.GetHandle().name();
            if (layer == grasplink::physics::CollisionLayer::Environment &&
                entityName != nullptr && std::strcmp(entityName, "Base_CollisionProxy") == 0)
            {
                m_BaseEnvironmentBodyHandle = physicsSystem.GetBodyHandle(entity.GetHandle());
            }
            if (layer == grasplink::physics::CollisionLayer::Robot ||
                layer == grasplink::physics::CollisionLayer::Gripper)
                m_CollisionEntities.push_back(entity.GetHandle());
        }
        const auto children = entity.GetChildren();
        pending.insert(pending.end(), children.begin(), children.end());
    }

    m_CollisionBodyHandles.reserve(m_CollisionEntities.size());
    for (const flecs::entity entity : m_CollisionEntities)
        m_CollisionBodyHandles.push_back(physicsSystem.GetBodyHandle(entity));

    m_RobotController.SetJointPoseCollisionValidator([this](const grasplink::robotics::JointVector& candidateJoints)
    {
        return ValidateCandidatePose(candidateJoints);
    });
    m_ValidatorRegistered = true;
}

void ViewerRobotCollisionGuard::CaptureSafeJointPose()
{
    const auto& currentJoints = m_RobotController.GetStateView().jointPositionRadians;
    std::copy(currentJoints.begin(), currentJoints.end(), m_SafeJointPositions.begin());
}

void ViewerRobotCollisionGuard::RestoreSafePoseIfOverlapping()
{
    if (m_RobotController.GetStateView().mode != grasplink::robotics::RobotMode::Moving ||
        !RobotAssemblyOverlapsEnvironment())
        return;

    // Jolt 접촉 callback은 Kinematic 로봇 부품과 Static 바닥의 겹침을 항상 알리지 않으므로 물리 계산 전에 직전 안전 자세를 복원한다.
    if (!m_RobotController.RestoreCollisionSafeState(m_SafeJointPositions))
        return;

    ApplyJointPose(m_RobotController.GetStateView());
    TransformSystemModule::UpdateWorldTransforms(m_World);
}

void ViewerRobotCollisionGuard::ClearControllerValidator() noexcept
{
    if (!m_ValidatorRegistered)
        return;

    m_RobotController.SetJointPoseCollisionValidator({});
    m_ValidatorRegistered = false;
}

bool ViewerRobotCollisionGuard::ValidateCandidatePose(
    const grasplink::robotics::JointVector& candidateJoints)
{
    const auto& previousState = m_RobotController.GetStateView();
    m_CandidateState.jointPositionRadians = candidateJoints;
    ApplyJointPose(m_CandidateState);
    const bool collisionFree = !RobotAssemblyOverlapsEnvironment();
    ApplyJointPose(previousState);
    return collisionFree;
}

void ViewerRobotCollisionGuard::ApplyJointPose(const grasplink::robotics::RobotState& state)
{
    const auto& kinematicState = m_RobotKinematics.Update(state);
    m_RobotTransformAdapter.Apply(kinematicState);
    m_RobotPhysicsAdapter.Apply(kinematicState);
    const auto& gripperState = m_GripperKinematics.Update(m_GripperController.GetState());
    m_GripperTransformAdapter.Apply(gripperState);
    TransformSystemModule::UpdateWorldTransforms(m_World);
}

bool ViewerRobotCollisionGuard::RobotAssemblyOverlapsEnvironment() const
{
    for (std::size_t i = 0; i < m_CollisionEntities.size(); ++i)
    {
        const Entity entity(m_CollisionEntities[i]);
        const glm::mat4 worldMatrix = entity.GetWorldMatrix();
        glm::mat3 axes(worldMatrix);
        for (int axis = 0; axis < 3; ++axis)
            axes[axis] = glm::normalize(axes[axis]);

        grasplink::physics::Transform target;
        target.position = glm::vec3(worldMatrix[3]);
        target.rotation = glm::normalize(glm::quat_cast(axes));
        const flecs::entity collisionEntity = m_CollisionEntities[i];
        const char* entityName = collisionEntity.name();
        const bool isLink1 = entityName != nullptr &&
            std::strcmp(entityName, "Link1_CollisionProxy") == 0;
        const bool overlapsEnvironment = isLink1 && m_BaseEnvironmentBodyHandle.IsValid()
            ? m_PhysicsWorld.OverlapsEnvironmentAt(
                m_CollisionBodyHandles[i], target, m_BaseEnvironmentBodyHandle)
            : m_PhysicsWorld.OverlapsEnvironmentAt(m_CollisionBodyHandles[i], target);
        if (overlapsEnvironment)
            return true;
    }
    return false;
}
}
