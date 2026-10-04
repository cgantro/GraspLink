#pragma once

#include "control/specs/DeviceSpecifications.h"

#include <array>

namespace control::specs::robotiq2f85
{

inline constexpr double kNominalClosedMasterRadians = 0.7929;
inline constexpr Axis3 kJointAxis{0.0, 0.0, -1.0};
inline constexpr double kJointPlaneZ = 0.0934257339;

inline constexpr Vec3 kLeftOuterPivot {-0.03060114, 0.05490452, kJointPlaneZ};
inline constexpr Vec3 kRightOuterPivot{+0.03060114, 0.05490452, kJointPlaneZ};
inline constexpr Vec3 kLeftInnerPivot {-0.01270000, 0.06142000, kJointPlaneZ};
inline constexpr Vec3 kRightInnerPivot{+0.01270000, 0.06142000, kJointPlaneZ};
inline constexpr Vec3 kLeftTipPivot   {-0.06775864, 0.09832620, kJointPlaneZ};
inline constexpr Vec3 kRightTipPivot  {+0.06775864, 0.09832620, kJointPlaneZ};

/**
 * @brief 2F-85 free-space mimic linkage.
 * @note 접촉 이후 under-actuated 적응 동작은 Physics 계층의 책임이다.
 */
inline constexpr std::array<GripperJointSpecification, 6> kJoints{{
    {"LeftOuterKnuckleJoint",  kLeftOuterPivot,  kJointAxis, +1.0,  0.0,  0.8, true},
    {"RightOuterKnuckleJoint", kRightOuterPivot, kJointAxis, -1.0, -0.8,  0.0, false},
    {"LeftInnerKnuckleJoint",  kLeftInnerPivot,  kJointAxis, +1.0,  0.0,  0.8, false},
    {"RightInnerKnuckleJoint", kRightInnerPivot, kJointAxis, -1.0, -0.8,  0.0, false},
    {"LeftFingerTipJoint",     kLeftTipPivot,    kJointAxis, -1.0, -0.8,  0.0, false},
    {"RightFingerTipJoint",    kRightTipPivot,   kJointAxis, +1.0,  0.0,  0.8, false},
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
