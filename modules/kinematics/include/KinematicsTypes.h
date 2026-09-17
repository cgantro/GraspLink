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
 * 이 타입의 position은 부모(A) 좌표계에서 본 자식(B) 원점이고, orientation은 B에서 A로의
 * 능동 회전을 나타내는 단위 quaternion이다. 길이 단위는 metre, 각도 단위는 radian이다.
 *
 * Pose.h의 Pose와 표현은 같지만, 여기서는 변환의 "from/to frame" 의미가 중요하므로
 * 기구학/파지 API에서 RigidTransform이라는 이름을 사용한다. 이 타입은 OpenGL glm 등을
 * 포함하지 않아 firmware host test와 simulator 모두에서 재사용할 수 있다.
 */
struct RigidTransform
{
    Position3D position{};
    Quaternion orientation{};
};

/**
 * 공통 domain의 JointState를 그대로 사용한다. J1부터 J6 순서, 단위 radian이라는
 * wire/domain contract를 기구학 모듈이 다시 정의하지 않아 transport와 simulator 사이의
 * 동일한 joint state가 유지된다. DLS 내부 연산은 double로 승격해 수행한다.
 */
using JointState = ::PoseLink::JointState;

/** IK 또는 command pipeline이 보고하는 로봇의 관측 가능한 상태이다. */
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
 * \brief 두 pose의 차이를 controller가 사용할 수 있는 6차원 error로 표현한다.
 *
 * positionError는 target origin - current origin [m]이다. rotationError는
 * \f$R_{error}=R_{target}R_{current}^{-1}\f$ 의 axis-angle vector [rad]이다.
 * 즉 direction은 right-handed rotation axis, magnitude는 [0, pi] 최단 회전각이다.
 */
struct PoseError
{
    Position3D positionError{};
    Position3D rotationError{};
};
} // namespace PoseLink::Kinematics
