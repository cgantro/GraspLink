#pragma once

#include "robotics/models/GripperSpecification.h"

#include <array>

namespace grasplink::robotics::models::robotiq
{

/**
 * @file TwoF85.h
 * @brief Robotiq 2F-85의 controller/simulation 공통 정적 규격.
 *
 * @details
 * 이 모델은 GLB animation을 사용하지 않고 runtime command로 linkage joint를 회전시키는 구조를 전제로 한다.
 * 현재 수치는 공개 2F-85 kinematic model의 joint frame을 controller-ready GLB의 gripper local frame으로
 * 대응시킨 free-space linkage 기준이다.
 *
 * 실제 2F-85는 under-actuated adaptive gripper이므로 물체 접촉 이후의 passive linkage 재배치는
 * 고정 mimic 비율만으로 정확히 표현할 수 없다. 현재 specification은 접촉 전 자유 공간 opening/closing용이며,
 * 접촉 이후 거동은 향후 Jolt Physics constraint/contact 계층에서 처리한다.
 */

/**
 * @brief 완전 닫힘 free-space 자세의 master linkage angle [rad].
 *
 * @note 0.7929 rad ~= 45.43 deg. 실제 내부 motor shaft angle이 아니라 simulation의 master joint 기준각이다.
 */
inline constexpr double kTwoF85NominalClosedMasterRadians = 0.7929;

/**
 * @brief controller-ready GLB의 gripper linkage joint local rotation axis.
 * @note 모든 moving gripper joint는 bind rotation identity이고 local -Z를 회전축으로 사용한다.
 */
inline constexpr Axis3 kTwoF85JointAxis{0.0, 0.0, -1.0};

/**
 * @brief 기존 gripper geometry와 공개 kinematic frame을 정렬할 때 사용한 공통 joint plane Z [m].
 */
inline constexpr double kTwoF85JointPlaneZ = 0.0934257339;

/** @brief Left outer knuckle pivot, Gripper local frame [m]. */
inline constexpr Vec3 kTwoF85LeftOuterPivot {-0.03060114, 0.05490452, kTwoF85JointPlaneZ};
/** @brief Right outer knuckle pivot, Gripper local frame [m]. */
inline constexpr Vec3 kTwoF85RightOuterPivot{+0.03060114, 0.05490452, kTwoF85JointPlaneZ};
/** @brief Left inner knuckle pivot, Gripper local frame [m]. */
inline constexpr Vec3 kTwoF85LeftInnerPivot {-0.01270000, 0.06142000, kTwoF85JointPlaneZ};
/** @brief Right inner knuckle pivot, Gripper local frame [m]. */
inline constexpr Vec3 kTwoF85RightInnerPivot{+0.01270000, 0.06142000, kTwoF85JointPlaneZ};
/** @brief Left fingertip linkage pivot, Gripper local frame [m]. */
inline constexpr Vec3 kTwoF85LeftTipPivot   {-0.06775864, 0.09832620, kTwoF85JointPlaneZ};
/** @brief Right fingertip linkage pivot, Gripper local frame [m]. */
inline constexpr Vec3 kTwoF85RightTipPivot  {+0.06775864, 0.09832620, kTwoF85JointPlaneZ};

/**
 * @brief 2F-85 free-space linkage의 master/mimic joint 관계.
 *
 * @details
 * Master angle을 q라고 하면 `jointAngle = masterMultiplier * q`로 자유 공간 자세를 계산한다.
 * q=0 rad가 fully open, q~=0.7929 rad가 nominal fully closed다.
 *
 * 각 항목 순서:
 * `{name, bindPivot[m], localAxis, masterMultiplier, minAngle[rad], maxAngle[rad], actuatorMaster}`
 *
 * 좌/우 대칭 구조 때문에 오른쪽 outer/inner와 왼쪽 fingertip은 부호가 반대다.
 * 이 배열은 "하나의 command -> 여러 linkage joint" 관계를 표현할 뿐 독립 motor 6개를 의미하지 않는다.
 */
inline constexpr std::array<GripperJointSpecification, 6> kTwoF85Joints{{
    {"LeftOuterKnuckleJoint",  kTwoF85LeftOuterPivot,  kTwoF85JointAxis, +1.0,  0.0,  0.8, true},
    {"RightOuterKnuckleJoint", kTwoF85RightOuterPivot, kTwoF85JointAxis, -1.0, -0.8,  0.0, false},
    {"LeftInnerKnuckleJoint",  kTwoF85LeftInnerPivot,  kTwoF85JointAxis, +1.0,  0.0,  0.8, false},
    {"RightInnerKnuckleJoint", kTwoF85RightInnerPivot, kTwoF85JointAxis, -1.0, -0.8,  0.0, false},
    {"LeftFingerTipJoint",     kTwoF85LeftTipPivot,    kTwoF85JointAxis, -1.0, -0.8,  0.0, false},
    {"RightFingerTipJoint",    kTwoF85RightTipPivot,   kTwoF85JointAxis, +1.0,  0.0,  0.8, false},
}};

/**
 * @brief Robotiq 2F-85 모델 전체 규격.
 *
 * @details
 * position/speed/force request 범위는 모두 0..255다.
 * position은 0=fully open, 255=fully closed이며 물리 거리나 angle 자체가 아니다.
 * 향후 SimGripperController는 `rPR -> master q -> mimic joints` 순서로 변환하고,
 * Hardware backend는 같은 값들을 장치 protocol의 rPR/rSP/rFR에 매핑한다.
 */
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
