#pragma once

#include "control/specs/DeviceSpecifications.h"

#include <array>

namespace control::specs::hcr12a
{

/**
 * @brief HCR-12A J1~J6 runtime joint specification.
 *
 * axis는 controller-ready GLB의 Joint local frame 기준이다.
 * 위치/속도 limit은 HCR-12A simulation specification에 기록한 값과 동일하다.
 */
inline constexpr std::array<JointSpecification, 6> kJoints{{
    {"J1", {0.0, 1.0, 0.0}, -3.141593,  3.141593, 2.268928},
    {"J2", {1.0, 0.0, 0.0}, -2.879793,  2.356194, 2.268928},
    {"J3", {1.0, 0.0, 0.0}, -1.483530,  4.276057, 3.490659},
    {"J4", {0.0, 0.0, 1.0}, -3.316126,  3.316126, 3.490659},
    {"J5", {1.0, 0.0, 0.0}, -2.967060,  2.967060, 3.490659},
    {"J6", {0.0, 0.0, 1.0}, -6.283185,  6.283185, 3.490659},
}};

inline constexpr RobotSpecification kSpecification{
    "Hanwha Robotics",
    "HCR-12A",
    kJoints.data(),
    kJoints.size()};

} // namespace control::specs::hcr12a
