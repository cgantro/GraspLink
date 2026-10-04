#pragma once

#include "robotics/models/GripperSpecification.h"

#include <array>

namespace grasplink::robotics::models::robotiq
{

/*
 * [추가 용어 설명]
 * - Knuckle: 손가락 링크가 연결되는 마디/힌지 부분.
 * - Fingertip joint: 손가락 끝쪽 링크가 회전하는 관절.
 * - Master angle q: 여러 linkage joint를 계산할 때 기준으로 삼는 대표 관절각.
 * - Mimic linkage: 각 관절을 따로 명령하지 않고 master q를 +1/-1 같은 계수로 따라 움직이게 하는 관계.
 * - Under-actuated: 관절 수보다 구동기 수가 적어서 물체 접촉 후 일부 관절이 수동적으로 적응하는 구조.
 * - Free-space: 물체 접촉 없이 공중에서 여닫는 상태. 아래 고정 mimic 관계는 이 상태를 기준으로 한다.
 */

// Free-space에서 nominal fully-closed 자세를 표현하는 master linkage 각도 [rad].
// 0.7929 rad ~= 45.43 deg. 실제 내부 motor shaft angle이 아니다.
inline constexpr double kTwoF85NominalClosedMasterRadians = 0.7929;

// 현재 controller-ready GLB에서 모든 moving gripper joint가 사용하는 local 회전축.
// {0,0,-1}은 각 Joint 자신의 local -Z축을 중심으로 회전한다는 뜻이다.
inline constexpr Axis3 kTwoF85JointAxis{0.0, 0.0, -1.0};

// 현재 GLB의 Gripper-local 좌표계에서 linkage hinge들이 놓인 공통 Z 위치 [m].
// 약 0.09343 m = 93.43 mm이며 제조사 모터 사양이 아니라 현재 asset에 맞춘 좌표값이다.
inline constexpr double kTwoF85JointPlaneZ = 0.0934257339;

// 각 값은 Gripper root 기준 [x,y,z] 회전 중심 [m].
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
/*
 * [추가 값 해설]
 * 각 항목 순서:
 * {name, pivot[m], localAxis, masterMultiplier, minAngle[rad], maxAngle[rad], actuatorMaster}
 *
 * master q가 0.4 rad라면:
 * - multiplier +1.0 -> +0.4 rad
 * - multiplier -1.0 -> -0.4 rad
 *
 * 좌/우가 거울 구조이기 때문에 일부 Joint는 부호가 반대다.
 * 이 배열에 Joint가 6개 있다고 해서 모터가 6개라는 뜻은 아니다.
 * 하나의 대표 actuator/master 움직임을 여러 링크가 기구학적으로 따라가는 관계를 표현한다.
 */
inline constexpr std::array<GripperJointSpecification, 6> kTwoF85Joints{{
    // Left outer knuckle: master q와 같은 방향. 0~0.8 rad. 자유공간 master 기준 관절.
    {"LeftOuterKnuckleJoint",  kTwoF85LeftOuterPivot,  kTwoF85JointAxis, +1.0,  0.0,  0.8, true},

    // Right outer knuckle: 좌우 대칭 때문에 master q와 반대 방향. -0.8~0 rad.
    {"RightOuterKnuckleJoint", kTwoF85RightOuterPivot, kTwoF85JointAxis, -1.0, -0.8,  0.0, false},

    // Left inner knuckle: master q와 같은 방향으로 연동.
    {"LeftInnerKnuckleJoint",  kTwoF85LeftInnerPivot,  kTwoF85JointAxis, +1.0,  0.0,  0.8, false},

    // Right inner knuckle: master q와 반대 방향으로 연동.
    {"RightInnerKnuckleJoint", kTwoF85RightInnerPivot, kTwoF85JointAxis, -1.0, -0.8,  0.0, false},

    // Left fingertip linkage: outer knuckle과 반대 부호로 회전해 손가락 링크 자세를 맞춘다.
    {"LeftFingerTipJoint",     kTwoF85LeftTipPivot,    kTwoF85JointAxis, -1.0, -0.8,  0.0, false},

    // Right fingertip linkage: 오른쪽 outer와 반대 부호(+q)로 회전한다.
    {"RightFingerTipJoint",    kTwoF85RightTipPivot,   kTwoF85JointAxis, +1.0,  0.0,  0.8, false},
}};

// 2F-85 모델 전체 규격을 하나로 묶는다.
// 뒤의 0,255 쌍은 순서대로 position, speed, force raw request의 최소/최대 범위다.
// position 0=open, 255=closed이며, 이 값 자체가 opening width[mm] 또는 joint angle[rad]은 아니다.
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
