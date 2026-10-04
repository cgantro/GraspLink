#pragma once

#include "control/specs/DeviceSpecifications.h"

#include <array>

namespace control::specs::robotiq2f85
{

inline constexpr double kNominalClosedMasterRadians = 0.7929;
inline constexpr Axis3 kJointAxis{0.0, 0.0, -1.0};

/**
 * @brief 2F-85 free-space mimic linkage.
 * @note 접촉 이후 under-actuated 적응 동작은 Physics 계층의 책임이다.
 */
inline constexpr std::array<GripperJointSpecification, 6> kJoints{{
    {"LeftOuterKnuckleJoint",  kJointAxis, +1.0,  0.0,  0.8, true},
    {"RightOuterKnuckleJoint", kJointAxis, -1.0, -0.8,  0.0, false},
    {"LeftInnerKnuckleJoint",  kJointAxis, +1.0,  0.0,  0.8, false},
    {"RightInnerKnuckleJoint", kJointAxis, -1.0, -0.8,  0.0, false},
    {"LeftFingerTipJoint",     kJointAxis, -1.0, -0.8,  0.0, false},
    {"RightFingerTipJoint",    kJointAxis, +1.0,  0.0,  0.8, false},
}};

inline constexpr GripperSpecification kSpecification{
    "Robotiq",
    "2F-85",
    0, 255,
    0, 255,
    0, 255,
    kNominalClosedMasterRadians,
    kJoints.data(),
    kJoints.size()};

} // namespace control::specs::robotiq2f85
