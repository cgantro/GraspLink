#pragma once

#include "robotics/models/GripperSpecification.h"

#include <array>

namespace grasplink::robotics::models::robotiq
{

inline constexpr double kTwoF85NominalClosedMasterRadians = 0.7929;
inline constexpr Axis3 kTwoF85JointAxis{0.0, 0.0, -1.0};
inline constexpr double kTwoF85JointPlaneZ = 0.0934257339;

inline constexpr Vec3 kTwoF85LeftOuterPivot {-0.03060114, 0.05490452, kTwoF85JointPlaneZ};
inline constexpr Vec3 kTwoF85RightOuterPivot{+0.03060114, 0.05490452, kTwoF85JointPlaneZ};
inline constexpr Vec3 kTwoF85LeftInnerPivot {-0.01270000, 0.06142000, kTwoF85JointPlaneZ};
inline constexpr Vec3 kTwoF85RightInnerPivot{+0.01270000, 0.06142000, kTwoF85JointPlaneZ};
inline constexpr Vec3 kTwoF85LeftTipPivot   {-0.06775864, 0.09832620, kTwoF85JointPlaneZ};
inline constexpr Vec3 kTwoF85RightTipPivot  {+0.06775864, 0.09832620, kTwoF85JointPlaneZ};

/**
 * @brief 2F-85 free-space mimic linkage.
 * @note 접촉 이후 under-actuated 적응 동작은 Physics 계층의 책임이다.
 */
inline constexpr std::array<GripperJointSpecification, 6> kTwoF85Joints{{
    {"LeftOuterKnuckleJoint",  kTwoF85LeftOuterPivot,  kTwoF85JointAxis, +1.0,  0.0,  0.8, true},
    {"RightOuterKnuckleJoint", kTwoF85RightOuterPivot, kTwoF85JointAxis, -1.0, -0.8,  0.0, false},
    {"LeftInnerKnuckleJoint",  kTwoF85LeftInnerPivot,  kTwoF85JointAxis, +1.0,  0.0,  0.8, false},
    {"RightInnerKnuckleJoint", kTwoF85RightInnerPivot, kTwoF85JointAxis, -1.0, -0.8,  0.0, false},
    {"LeftFingerTipJoint",     kTwoF85LeftTipPivot,    kTwoF85JointAxis, -1.0, -0.8,  0.0, false},
    {"RightFingerTipJoint",    kTwoF85RightTipPivot,   kTwoF85JointAxis, +1.0,  0.0,  0.8, false},
}};

inline constexpr GripperSpecification kTwoF85{
    "Robotiq",
    "2F-85",
    0, 255,
    0, 255,
    0, 255,
    kTwoF85NominalClosedMasterRadians,
    kTwoF85Joints.data(),
    kTwoF85Joints.size()};

} // namespace grasplink::robotics::models::robotiq
