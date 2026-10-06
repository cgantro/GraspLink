#pragma once

#include "robotics/models/GripperSpecification.h"

#include <array>

namespace grasplink::robotics::models::robotiq
{

/**
 * @brief 공개된 2F-85 기구 기준 자료에서 자유공간 닫힘 상태를 나타내는 기준 관절 각 [rad]. 실제 모터축 각도나 접촉 후 파지 위치가 아니다.
 * @details linkage 기준 master 관절각이며 Robotiq 장치의 내부 motor shaft 각도가 아니다. 물체 접촉 뒤 손가락의 under-actuated 적응을 기술하지 않는다.
 */
inline constexpr double kTwoF85NominalClosedMasterRadians = 0.7929;

/**
 * @brief 현재 GLB 관절 초기 자세에서 읽어 온 각 관절의 Local 회전축 방향.
 * @details 단위 방향값이며 Joint Node의 local frame 기준이다. 다른 모델 asset의 축으로 일반화하지 않는다.
 */
inline constexpr Axis3 kTwoF85JointAxis{0.0, 0.0, -1.0};

/**
 * @brief 아래 회전 중심들과 함께 사용하며 현재 GLB 모델 좌표에서 읽은 관절 회전 평면의 Z 위치 [m].
 * @details 좌표는 GLB의 Gripper specification root 기준이며 아래 joint node 자체의 local 원점은 아니다.
 * 현재 Gripper root에는 비항등 bind transform이 있고, 중첩된 joint node들의 bind 위치도 이 root 기준값과 다를 수 있다.
 * 따라서 이 상수는 현재 GLB에만 적용되며 다른 GLB에 그대로 사용할 수 없다.
 */
inline constexpr double kTwoF85JointPlaneZ = 0.0934257339;

/**
 * @brief 현재 GLB의 Gripper 사양 root를 기준으로 측정한 관절 회전 중심 위치 [m].
 * @details pivot은 asset-derived 참조값이다. GLB root의 비항등 bind transform 때문에 실제 node별 nested bind translation과 같다고 볼 수 없다.
 */
inline constexpr Vec3 kTwoF85LeftOuterPivot {-0.03060114, 0.05490452, kTwoF85JointPlaneZ};
inline constexpr Vec3 kTwoF85RightOuterPivot{+0.03060114, 0.05490452, kTwoF85JointPlaneZ};
inline constexpr Vec3 kTwoF85LeftInnerPivot {-0.01270000, 0.06142000, kTwoF85JointPlaneZ};
inline constexpr Vec3 kTwoF85RightInnerPivot{+0.01270000, 0.06142000, kTwoF85JointPlaneZ};
inline constexpr Vec3 kTwoF85LeftTipPivot   {-0.06775864, 0.09832620, kTwoF85JointPlaneZ};
inline constexpr Vec3 kTwoF85RightTipPivot  {+0.06775864, 0.09832620, kTwoF85JointPlaneZ};

/**
 * @brief 물체와 접촉하지 않는 경우 기준 관절 각으로부터 여섯 linkage 관절 각을 계산하는 고정 비율.
 * @details
 * pivot은 현재 GLB의 Gripper specification root 기준 [m], axis는 각 Joint Node의 local 방향이다.
 * 이 관계는 열린 공간에서 기구학을 구동하기 위한 참조다. 물체 접촉 뒤 under-actuated 적응과 접촉력 계산은 이 배열의 책임이 아니다.
 * 각 원소: 이름, root 기준 pivot [m], local axis, master 계수, 허용각 [rad], 기준 관절 여부.
 */
inline constexpr std::array<GripperJointSpecification, 6> kTwoF85Joints{{
    // 왼쪽 바깥 너클은 기준 관절이며 자유공간에서 master 관절과 같은 방향으로 회전한다.
    {"LeftOuterKnuckleJoint",  kTwoF85LeftOuterPivot,  kTwoF85JointAxis, +1.0,  0.0,  0.8, true},

    // 오른쪽 바깥 너클은 왼쪽과 대칭이어서 master 관절과 반대 방향으로 회전한다.
    {"RightOuterKnuckleJoint", kTwoF85RightOuterPivot, kTwoF85JointAxis, -1.0, -0.8,  0.0, false},

    // 왼쪽 안쪽 너클은 자유공간에서 master 관절과 같은 방향으로 회전한다.
    {"LeftInnerKnuckleJoint",  kTwoF85LeftInnerPivot,  kTwoF85JointAxis, +1.0,  0.0,  0.8, false},

    // 오른쪽 안쪽 너클은 자유공간에서 master 관절과 반대 방향으로 회전한다.
    {"RightInnerKnuckleJoint", kTwoF85RightInnerPivot, kTwoF85JointAxis, -1.0, -0.8,  0.0, false},

    // 왼쪽 손가락 끝 관절은 같은 쪽 바깥 너클과 반대 방향으로 회전한다.
    {"LeftFingerTipJoint",     kTwoF85LeftTipPivot,    kTwoF85JointAxis, -1.0, -0.8,  0.0, false},

    // 오른쪽 손가락 끝 관절은 같은 쪽 바깥 너클과 반대 방향으로 회전한다.
    {"RightFingerTipJoint",    kTwoF85RightTipPivot,   kTwoF85JointAxis, +1.0,  0.0,  0.8, false},
}};

/**
 * @brief Robotiq 2F-85의 장치 요청 code 범위와 자유공간 linkage 관절 계산에 필요한 모델 상수를 제공한다.
 * @details
 * rPR 0..255는 제조사 프로토콜 값이며 0은 open, 255는 closed 요청이다. 값 자체는 [m]이나 [rad]가 아니다.
 * rSP와 rFR도 장치 프로토콜 code이며 실제 속도 [mm/s]나 힘 [N]을 뜻하지 않는다.
 * 이 사양은 장치 register 변환을 하거나 접촉한 물체에 맞춰 손가락을 움직이지 않는다.
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
