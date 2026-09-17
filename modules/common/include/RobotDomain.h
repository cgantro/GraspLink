#pragma once

#include "Pose.h"

#include <array>
#include <cstdint>

namespace PoseLink
{
/**
 * @brief 사람 controller에서 simulator로 보내는 translation-only command.
 *
 * `position`은 simulator robot-base frame 기준 metre다. controller가 orientation
 * 을 보내지 않는 것은 의도된 v1 제약이다. simulator는 이 XYZ와 설정된 fixed TCP
 * orientation을 합쳐 full 6-DoF IK target을 만든다. `sequence`는 wrap하는 단조
 * 증가 uint32이고 receiver가 modular 비교로 신선도를 판정한다. `targetId`는
 * movable scene object를 가리키며 single-object demo 이후도 호환 가능하게 한다.
 */
struct TargetCommand
{
    std::uint32_t sequence = 0U;
    std::uint32_t targetId = 0U;
    Position3D position{};
};

/**
 * @brief physical-contact simulation이 아닌 관측 가능한 grasp/IK 결과.
 *
 * `Grasped`는 TCP가 설정된 position *및* orientation tolerance에 모두 들어와
 * object frame이 TCP에 attach된 상태다. `IkUnreachable`은 미완성 numerical
 * result를 적용하지 않고 마지막 safe joint state를 보존한다.
 */
enum class GraspStatus : std::uint8_t
{
    Idle = 0U,
    Tracking = 1U,
    Grasped = 2U,
    IkUnreachable = 3U,
    ProtocolError = 4U,
};

/**
 * @brief Simulator-to-controller acknowledgement for a received target.
 *
 * `sequence` orders outgoing state reports independently. The acknowledged
 * command sequence prevents delayed UDP feedback being mistaken for the most
 * recent button press on the ESP32.
 */
struct RobotState
{
    std::uint32_t sequence = 0U;
    std::uint32_t acknowledgedTargetSequence = 0U;
    GraspStatus status = GraspStatus::Idle;
};

/**
 * @brief Six revolute-joint angles in radians, ordered J1 through J6.
 *
 * The state intentionally contains no renderer matrix or degree conversion:
 * keeping robotics quantities in SI radians prevents screen transforms from
 * accidentally being re-used as robot commands.
 */
struct JointState
{
    std::array<float, 6U> radians{};
};
} // namespace PoseLink
