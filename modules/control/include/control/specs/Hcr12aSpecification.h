#pragma once

#include "control/specs/DeviceSpecifications.h"

#include <array>

namespace control::specs::hcr12a
{

/**
 * @brief HCR-12A J1~J6 runtime joint specification.
 *
 * bindPivotMeters는 controller-ready GLB/CAD bind pose 기준 world pivot이다.
 * axis는 해당 Joint local frame 기준이다.
 */
inline constexpr std::array<JointSpecification, 6> kJoints{{
    {"J1", { 0.0000, 0.1985, 0.00000}, {0.0, 1.0, 0.0}, -3.141593,  3.141593, 2.268928},
    {"J2", { 0.1535, 0.3100, 0.10000}, {1.0, 0.0, 0.0}, -2.879793,  2.356194, 2.268928},
    {"J3", { 0.1095, 0.9100, 0.10000}, {1.0, 0.0, 0.0}, -1.483530,  4.276057, 3.490659},
    {"J4", { 0.0000, 1.0150, 0.23350}, {0.0, 0.0, 1.0}, -3.316126,  3.316126, 3.490659},
    {"J5", {-0.1385, 1.0150, 0.69100}, {1.0, 0.0, 0.0}, -2.967060,  2.967060, 3.490659},
    {"J6", { 0.0000, 1.0150, 0.85475}, {0.0, 0.0, 1.0}, -6.283185,  6.283185, 3.490659},
}};

inline constexpr Vec3 kToolFrameBindWorldMeters{0.0, 1.0150, 0.9145};

inline constexpr RobotSpecification kSpecification{
    "Hanwha Robotics",
    "HCR-12A",
    kJoints.data(),
    kJoints.size()};

} // namespace control::specs::hcr12a
