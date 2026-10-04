#pragma once

#include "robotics/models/GripperSpecification.h"

#include <array>

namespace grasplink::robotics::models::robotiq
{

/**
 * @file TwoF85.h
 * @brief Robotiq 2F-85를 시뮬레이션/Controller에서 해석하기 위한 모델별 고정값.
 *
 * @details
 * 이 파일은 그리퍼를 실제로 움직이는 Controller가 아니다.
 * Controller가 사용할 "2F-85라는 모델의 규칙과 숫자"를 한 곳에 모은 사양 테이블이다.
 *
 * 초보자용 용어:
 * - Knuckle: 손가락 뿌리 쪽의 회전 링크/마디.
 * - Fingertip: 물체와 가까이 접촉하는 손가락 끝 링크.
 * - Linkage: 여러 링크/관절이 기계적으로 연결되어 함께 움직이는 구조.
 * - Master angle q: 여러 관절 움직임을 계산할 때 기준으로 쓰는 대표 각도.
 * - Mimic: 독립적으로 제어하지 않고 master angle을 +1/-1 등의 배율로 따라가는 관계.
 * - Free-space: 물체와 접촉하지 않은 상태에서 그리퍼가 그냥 열리고 닫히는 상황.
 * - Under-actuated: 관절 수보다 모터 수가 적어서, 물체와 닿은 뒤 일부 관절이 수동적으로 적응하는 구조.
 * - Local frame: Gripper/Joint 자체를 기준으로 보는 좌표계. World 좌표와 다르다.
 *
 * 현재 모델은 GLB animation을 재생하지 않는다.
 * 나중에 Controller가 rPR 같은 명령값을 master angle q로 바꾸고, q에서 각 joint angle을 계산해
 * GLB Joint transform을 직접 갱신하는 구조를 전제로 한다.
 *
 * 실제 2F-85는 under-actuated adaptive gripper이므로 물체 접촉 이후 동작은
 * 아래의 고정 mimic 관계만으로 완전히 표현할 수 없다. 접촉 이후는 Physics 계층의 책임이다.
 */

/**
 * @brief 자유공간에서 거의 완전히 닫힌 자세를 나타내는 master linkage 기준각 [rad].
 *
 * @details
 * 0.7929 rad는 약 45.43 deg다.
 * `q=0`을 open 기준으로 두고, `q≈0.7929`를 nominal closed 기준으로 사용한다.
 *
 * @warning 이 값은 Robotiq 내부 모터 shaft의 실제 회전각이 아니다.
 * 공개 기구학 모델에서 사용하는 시뮬레이션용 대표 joint angle이다.
 */
inline constexpr double kTwoF85NominalClosedMasterRadians = 0.7929;

/**
 * @brief 현재 controller-ready GLB에서 모든 그리퍼 moving joint가 회전하는 local 축.
 *
 * @details
 * `{0,0,-1}`은 각 Joint 자신의 local -Z 방향을 회전축으로 사용한다는 뜻이다.
 * 축은 방향만 나타내므로 meter/radian 같은 물리 단위가 없다.
 */
inline constexpr Axis3 kTwoF85JointAxis{0.0, 0.0, -1.0};

/**
 * @brief 현재 GLB의 Gripper local 좌표계에서 linkage joint들이 놓인 공통 Z 평면 위치 [m].
 *
 * @details
 * 0.0934257339 m ≈ 93.43 mm다.
 * 이 값은 2F-85 제품의 일반 공식 치수가 아니라, 공개 기구학 frame을 현재 GLB geometry에 맞춘 asset-specific 값이다.
 */
inline constexpr double kTwoF85JointPlaneZ = 0.0934257339;

/** @brief 왼쪽 outer knuckle의 회전 중심. Gripper local frame 기준 [m]. */
inline constexpr Vec3 kTwoF85LeftOuterPivot {-0.03060114, 0.05490452, kTwoF85JointPlaneZ};
/** @brief 오른쪽 outer knuckle의 회전 중심. Gripper local frame 기준 [m]. */
inline constexpr Vec3 kTwoF85RightOuterPivot{+0.03060114, 0.05490452, kTwoF85JointPlaneZ};
/** @brief 왼쪽 inner knuckle의 회전 중심. Gripper local frame 기준 [m]. */
inline constexpr Vec3 kTwoF85LeftInnerPivot {-0.01270000, 0.06142000, kTwoF85JointPlaneZ};
/** @brief 오른쪽 inner knuckle의 회전 중심. Gripper local frame 기준 [m]. */
inline constexpr Vec3 kTwoF85RightInnerPivot{+0.01270000, 0.06142000, kTwoF85JointPlaneZ};
/** @brief 왼쪽 fingertip linkage의 회전 중심. Gripper local frame 기준 [m]. */
inline constexpr Vec3 kTwoF85LeftTipPivot   {-0.06775864, 0.09832620, kTwoF85JointPlaneZ};
/** @brief 오른쪽 fingertip linkage의 회전 중심. Gripper local frame 기준 [m]. */
inline constexpr Vec3 kTwoF85RightTipPivot  {+0.06775864, 0.09832620, kTwoF85JointPlaneZ};

/**
 * @brief 2F-85가 자유공간에서 열리고 닫힐 때 6개 관절이 master angle을 따라가는 규칙.
 *
 * @details
 * 여기서 "mimic linkage"는 하나의 master angle q를 기준으로 다른 관절들이 정해진 부호/배율로 따라가는 관계다.
 * 즉 모터 6개를 각각 제어한다는 뜻이 아니다.
 *
 * 계산식:
 * `jointAngle = masterMultiplier * q`
 *
 * 예를 들어 q=0.4 rad라면:
 * - LeftOuter  = +0.4
 * - RightOuter = -0.4
 * - LeftInner  = +0.4
 * - RightInner = -0.4
 * - LeftTip    = -0.4
 * - RightTip   = +0.4
 *
 * 각 배열 원소의 순서:
 * `{name, bindPivot[m], localAxis, masterMultiplier, minAngle[rad], maxAngle[rad], actuatorMaster}`
 */
inline constexpr std::array<GripperJointSpecification, 6> kTwoF85Joints{{
    // 왼쪽 바깥 knuckle. master 기준 관절이며 q와 같은 방향으로 0~0.8 rad 회전한다.
    {"LeftOuterKnuckleJoint",  kTwoF85LeftOuterPivot,  kTwoF85JointAxis, +1.0,  0.0,  0.8, true},

    // 오른쪽 바깥 knuckle. 좌우 대칭이므로 q의 반대 방향으로 -0.8~0 rad 회전한다.
    {"RightOuterKnuckleJoint", kTwoF85RightOuterPivot, kTwoF85JointAxis, -1.0, -0.8,  0.0, false},

    // 왼쪽 안쪽 knuckle. master q와 같은 방향으로 따라간다.
    {"LeftInnerKnuckleJoint",  kTwoF85LeftInnerPivot,  kTwoF85JointAxis, +1.0,  0.0,  0.8, false},

    // 오른쪽 안쪽 knuckle. 대칭 구조라 q의 반대 방향으로 따라간다.
    {"RightInnerKnuckleJoint", kTwoF85RightInnerPivot, kTwoF85JointAxis, -1.0, -0.8,  0.0, false},

    // 왼쪽 fingertip linkage. 왼쪽 outer와 반대 방향으로 회전해 finger 자세를 형성한다.
    {"LeftFingerTipJoint",     kTwoF85LeftTipPivot,    kTwoF85JointAxis, -1.0, -0.8,  0.0, false},

    // 오른쪽 fingertip linkage. 오른쪽 outer와 반대 방향이므로 결과적으로 +q를 사용한다.
    {"RightFingerTipJoint",    kTwoF85RightTipPivot,   kTwoF85JointAxis, +1.0,  0.0,  0.8, false},
}};

/**
 * @brief 위의 2F-85 관련 값을 Controller가 한 번에 받을 수 있도록 묶은 전체 모델 사양.
 *
 * @details
 * 필드 순서와 의미:
 * - "Robotiq": 제조사 이름
 * - "2F-85": 모델 이름
 * - 0,255: position request 범위. 2F-85에서는 0=open, 255=closed
 * - 0,255: speed request 범위
 * - 0,255: force request 범위
 * - kTwoF85NominalClosedMasterRadians: 자유공간 closed 기준 master angle [rad]
 * - kTwoF85Joints.data(): 6개 linkage joint 사양 배열 주소
 * - kTwoF85Joints.size(): joint 개수, 현재 6
 *
 * Position/Speed/Force의 0..255 값은 mm, mm/s, N 자체가 아니라 장치의 raw command 값이다.
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
