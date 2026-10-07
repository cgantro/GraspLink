#include "ViewerRobotCollisionGuard.h"

#include "Entity.h"
#include "TransformSystemModule.h"

#include "PhysicsWorld.h"
#include "simulation/components/PhysicsComponents.h"
#include "simulation/components/RobotCollisionProxy.h"
#include "simulation/systems/PhysicsSystemModule.h"

#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/core/IGripperController.h"
#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/kinematics/GripperKinematics.h"

#include "viewer/robotics/RobotTransformAdapter.h"
#include "viewer/robotics/GripperTransformAdapter.h"
#include "simulation/robotics/RobotPhysicsAdapter.h"

#include <algorithm>
#include <glm/gtc/quaternion.hpp>
#if GRASPLINK_ENABLE_TRACY
#include <tracy/Tracy.hpp>
#else
#define ZoneScopedN(name) ((void)0)
#endif

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
      m_SafeJointPositions(robotController.GetStateView().jointPositionRadians),
      m_PhysicsSystem(physicsSystem)
{
    std::vector<Entity> pending{robotRoot};
    while (!pending.empty())
    {
        Entity entity = pending.back();
        pending.pop_back();
        if (entity.Has<RigidBody>())
        {
            const auto layer = entity.Get<RigidBody>().collisionLayer;
            if (entity.Has<RobotBaseEnvironmentProxy>())
                m_BaseEnvironmentEntity = entity.GetHandle();
            if (layer == grasplink::physics::CollisionLayer::Robot ||
                layer == grasplink::physics::CollisionLayer::Gripper)
                m_CollisionEntities.push_back(entity.GetHandle());
        }
        const auto children = entity.GetChildren();
        pending.insert(pending.end(), children.begin(), children.end());
    }

    m_RobotController.SetJointPoseCollisionValidator([this](const grasplink::robotics::JointVector& candidateJoints)
    {
        return ValidateCandidatePose(candidateJoints);
    });
}

ViewerRobotCollisionGuard::~ViewerRobotCollisionGuard()
{
    m_RobotController.SetJointPoseCollisionValidator({});
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
    ZoneScopedN("CollisionCheck");
    // Collider 설정 변경으로 Jolt Body가 재생성될 수 있으므로 매 검사마다 Entity에서 현재 handle을 조회한다.
    const auto baseEnvironmentBodyHandle = m_PhysicsSystem.GetBodyHandle(m_BaseEnvironmentEntity);
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
        const auto collisionBodyHandle = m_PhysicsSystem.GetBodyHandle(collisionEntity);
        // Physics Body가 없는 proxy는 겹침이 없다고 간주하지 않고 안전하지 않은 자세로 처리한다.
        if (!collisionBodyHandle.IsValid())
            return true;

        const char* entityName = collisionEntity.name();
        const bool isLink1 = entityName != nullptr &&
            std::strcmp(entityName, "Link1_CollisionProxy") == 0;
        const bool overlapsEnvironment = isLink1 && baseEnvironmentBodyHandle.IsValid()
            ? m_PhysicsWorld.OverlapsEnvironmentAt(
                collisionBodyHandle, target, baseEnvironmentBodyHandle)
            : m_PhysicsWorld.OverlapsEnvironmentAt(collisionBodyHandle, target);
        if (overlapsEnvironment)
            return true;
    }
    return false;
}
}
