#pragma once

#include "robotics/models/RobotSpecification.h"

#include <array>

namespace grasplink::robotics::models::hanwha
{

// HCR-12A runtime 상수.
// Pivot: STEP 체결 형상에서 구한 CAD 값, robot-base bind pose [m] (CAD mm / 1000).
// Axis: controller-ready GLB의 Joint-local 단위축. Bind rotation이 identity라 CAD 방향과 일치한다.
// Limit·속도: HCR-A (2G) 제조사 값, deg와 deg/s를 각각 rad와 rad/s로 변환.
// 각 항목: {이름, pivot[m], local axis, 최소각[rad], 최대각[rad], 최대속도[rad/s]}.
inline constexpr std::array<JointSpecification, 6> kHcr12aJoints{{
    // J1: base yaw.
    {"J1", { 0.0000, 0.1985, 0.00000}, {0.0, 1.0, 0.0}, -3.141593,  3.141593, 2.268928},

    // J2: shoulder pitch.
    {"J2", { 0.1535, 0.3100, 0.10000}, {1.0, 0.0, 0.0}, -2.879793,  2.356194, 2.268928},

    // J3: elbow pitch.
    {"J3", { 0.1095, 0.9100, 0.10000}, {1.0, 0.0, 0.0}, -1.483530,  4.276057, 3.490659},

    // J4: wrist roll.
    {"J4", { 0.0000, 1.0150, 0.23350}, {0.0, 0.0, 1.0}, -3.316126,  3.316126, 3.490659},

    // J5: wrist pitch.
    {"J5", {-0.1385, 1.0150, 0.69100}, {1.0, 0.0, 0.0}, -2.967060,  2.967060, 3.490659},

    // J6: 마지막 wrist roll.
    {"J6", { 0.0000, 1.0150, 0.85475}, {0.0, 0.0, 1.0}, -6.283185,  6.283185, 3.490659},
}};

inline const std::array<LinkSpecification, 6> kHcr12aLinks{{
    {"Link1", 0},
    {"Link2", 1},
    {"Link3", 2},
    {"Link4", 3},
    {"Link5", 4},
    {"Link6", 5},
}};

// Bind pose에서 측정한 ToolFrame의 robot-base 위치 [m]. FK 검증 기준점.
inline constexpr Vec3 kHcr12aToolFrameBindWorldMeters{0.0, 1.0150, 0.9145};

// ToolFrame 변환은 J6에서 고정이며, 장착 공구 TCP는 포함하지 않는다.
inline const RobotSpecification kHcr12a{
    "Hanwha Robotics",
    "HCR-12A",
    kHcr12aJoints.data(),
    kHcr12aJoints.size(),
    kHcr12aLinks.data(),
    kHcr12aLinks.size(),
    {{0.0, 0.0, kHcr12aToolFrameBindWorldMeters.z - kHcr12aJoints.back().bindPivotMeters.z}, {}},
    true};

} // namespace grasplink::robotics::models::hanwha
