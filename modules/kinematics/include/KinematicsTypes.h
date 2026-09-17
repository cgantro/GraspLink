#pragma once

#include "Pose.h"
#include "RobotDomain.h"

#include <cstdint>

namespace PoseLink::Kinematics
{
/**
 * \brief Renderer, physics engine, CAD library 어느 쪽에도 종속되지 않는 강체 변환이다.
 *
 * 좌표계 표기는 \f$T^A_B\f$ (B 좌표계의 점을 A 좌표계로 옮기는 변환)를 따른다.
 * position은 부모(A) 좌표계에서 본 자식(B) 원점이고, orientation은 B에서 A로의
 * 능동 회전을 나타내는 단위 quaternion이다. 길이 단위는 metre, 각도 단위는 radian이다.
 */
struct RigidTransform
{
    Position3D position{};
    Quaternion orientation{};
};

/** J1부터 J6 순서의 simulation joint state를 kinematics에서도 그대로 사용한다. */
using JointState = ::PoseLink::JointState;

/** IK/update pipeline이 보고하는 simulation 내부 상태이다. */
enum class RobotMotionStatus : std::uint8_t
{
    Idle,
    Tracking,
    TargetReached,
    IkUnreachable,
    IkIterationLimit,
    InvalidTarget
};

/** Object가 gripper/TCP에 고정되었는지를 나타낸다. */
enum class GraspState : std::uint8_t
{
    Open,
    Attached
};

/**
 * \brief 두 pose의 차이를 solver가 사용할 수 있는 6차원 error로 표현한다.
 *
 * positionError는 target origin - current origin [m]이다. rotationError는
 * \f$R_{error}=R_{target}R_{current}^{-1}\f$ 의 axis-angle vector [rad]이다.
 */
struct PoseError
{
    Position3D positionError{};
    Position3D rotationError{};
};
} // namespace PoseLink::Kinematics
