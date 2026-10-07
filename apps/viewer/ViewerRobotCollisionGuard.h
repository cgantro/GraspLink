#pragma once

#include "PhysicsTypes.h"
#include "robotics/core/ControlTypes.h"

#include <flecs.h>

#include <vector>

class Entity;

namespace grasplink::physics
{
class PhysicsWorld;
}

namespace grasplink::robotics
{
class IGripperController;
namespace backends::simulation { class SimRobotController; }
namespace kinematics { class RobotKinematics; class GripperKinematics; }
}

namespace grasplink::simulation
{
class PhysicsSystemModule;
class RobotPhysicsAdapter;
}

namespace grasplink::viewer::robotics
{
class RobotTransformAdapter;
class GripperTransformAdapter;
}

namespace grasplink::viewer
{
/**
 * @brief Viewer에서 로봇과 그리퍼의 Environment 충돌을 확인하고 안전한 관절 자세를 복원한다.
 * @details Environment는 바닥과 작업대처럼 로봇이 들어가면 안 되는 고정 물체다. 이 guard는 화면 Entity 자세를 Jolt 충돌 형상에 전달해 Controller가 제안한 자세와 고정 tick의 실제 자세를 검사한다.
 * Controller, 어댑터, Flecs World와 Physics 객체는 빌려 쓰므로 이 guard보다 오래 살아야 한다. Controller에 등록한 검사 함수는 guard가 파괴될 때 함께 해제된다.
 */
class ViewerRobotCollisionGuard final
{
public:
    ViewerRobotCollisionGuard(
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
        const Entity& robotRoot);
    ~ViewerRobotCollisionGuard();

    ViewerRobotCollisionGuard(const ViewerRobotCollisionGuard&) = delete;
    ViewerRobotCollisionGuard& operator=(const ViewerRobotCollisionGuard&) = delete;

    /** @brief 현재 Controller 관절각을 다음 고정 tick 충돌 검사에서 쓸 안전 자세로 저장한다. */
    void CaptureSafeJointPose();

    /** @brief 이동 중 현재 자세가 Environment와 겹치면 직전 안전 관절각과 Entity 변환으로 복원한다. */
    void RestoreSafePoseIfOverlapping();

private:
    [[nodiscard]] bool ValidateCandidatePose(const grasplink::robotics::JointVector& candidateJoints);
    void ApplyJointPose(const grasplink::robotics::RobotState& state);
    [[nodiscard]] bool RobotAssemblyOverlapsEnvironment() const;

    grasplink::robotics::backends::simulation::SimRobotController& m_RobotController;
    grasplink::robotics::IGripperController& m_GripperController;
    grasplink::robotics::kinematics::RobotKinematics& m_RobotKinematics;
    grasplink::viewer::robotics::RobotTransformAdapter& m_RobotTransformAdapter;
    grasplink::simulation::RobotPhysicsAdapter& m_RobotPhysicsAdapter;
    grasplink::robotics::kinematics::GripperKinematics& m_GripperKinematics;
    grasplink::viewer::robotics::GripperTransformAdapter& m_GripperTransformAdapter;
    flecs::world& m_World;
    grasplink::physics::PhysicsWorld& m_PhysicsWorld;
    grasplink::robotics::RobotState m_CandidateState;
    grasplink::robotics::JointVector m_SafeJointPositions;
    std::vector<flecs::entity> m_CollisionEntities;
    flecs::entity m_BaseEnvironmentEntity;
    grasplink::simulation::PhysicsSystemModule& m_PhysicsSystem;
};
}
