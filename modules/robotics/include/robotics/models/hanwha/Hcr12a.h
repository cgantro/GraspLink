#pragma once

#include "robotics/models/RobotSpecification.h"

#include <array>

namespace grasplink::robotics::models::hanwha
{

/**
 * @brief HCR-12A J1~J6 runtime specification.
 *
 * bindPivotMeters는 controller-ready GLB/CAD bind pose 기준 world pivot,
 * axis는 각 Joint local frame 기준이다.
 */
/*
 * [추가 용어/값 설명]
 * - J1~J6: Base에서 Tool 방향으로 번호를 붙인 6개의 회전 관절.
 * - bindPivotMeters: bind pose에서 각 관절이 실제로 도는 중심점 [m].
 * - axis: 해당 Joint 자신의 local 좌표계에서 본 회전축.
 * - min/maxPositionRadians: 관절 허용 각도 범위 [rad].
 * - maxVelocityRadiansPerSecond: 관절 허용 최대 각속도 [rad/s].
 *
 * 수치 변환:
 * - CAD pivot mm -> runtime m: mm / 1000
 * - 제품 각도 deg -> runtime rad: deg * pi / 180
 * - 제품 속도 deg/s -> runtime rad/s: deg/s * pi / 180
 *
 * 각 배열 항목의 순서는 다음과 같다.
 * {name, bindPivot[m], localAxis, minAngle[rad], maxAngle[rad], maxVelocity[rad/s]}
 */
inline constexpr std::array<JointSpecification, 6> kHcr12aJoints{{
    // J1: Base를 좌우로 돌리는 yaw 관절. Pivot=(0,198.5,0) mm, local +Y축 회전.
    {"J1", { 0.0000, 0.1985, 0.00000}, {0.0, 1.0, 0.0}, -3.141593,  3.141593, 2.268928},

    // J2: Shoulder를 앞/뒤로 기울이는 pitch 관절. Pivot=(153.5,310,100) mm, local +X축 회전.
    {"J2", { 0.1535, 0.3100, 0.10000}, {1.0, 0.0, 0.0}, -2.879793,  2.356194, 2.268928},

    // J3: Elbow를 접고 펴는 pitch 관절. Pivot=(109.5,910,100) mm, local +X축 회전.
    {"J3", { 0.1095, 0.9100, 0.10000}, {1.0, 0.0, 0.0}, -1.483530,  4.276057, 3.490659},

    // J4: Wrist를 축 방향으로 비트는 roll 관절. Pivot=(0,1015,233.5) mm, local +Z축 회전.
    {"J4", { 0.0000, 1.0150, 0.23350}, {0.0, 0.0, 1.0}, -3.316126,  3.316126, 3.490659},

    // J5: Wrist를 꺾는 pitch 관절. Pivot=(-138.5,1015,691) mm, local +X축 회전.
    {"J5", {-0.1385, 1.0150, 0.69100}, {1.0, 0.0, 0.0}, -2.967060,  2.967060, 3.490659},

    // J6: Tool/Gripper 전체를 축 방향으로 돌리는 마지막 roll 관절. Pivot=(0,1015,854.75) mm.
    {"J6", { 0.0000, 1.0150, 0.85475}, {0.0, 0.0, 1.0}, -6.283185,  6.283185, 3.490659},
}};

// Bind pose에서 ToolFrame의 world 위치 [m]. 향후 FK 결과 검증 시 기준점으로 사용할 수 있다.
inline constexpr Vec3 kHcr12aToolFrameBindWorldMeters{0.0, 1.0150, 0.9145};

// 위 관절 배열을 제조사/모델명과 함께 묶어 Controller/Viewer가 공유하는 모델 정의로 만든다.
inline constexpr RobotSpecification kHcr12a{
    "Hanwha Robotics",
    "HCR-12A",
    kHcr12aJoints.data(),
    kHcr12aJoints.size()};

} // namespace grasplink::robotics::models::hanwha
