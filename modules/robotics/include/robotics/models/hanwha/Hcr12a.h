#pragma once

#include "robotics/models/RobotSpecification.h"

#include <array>

namespace grasplink::robotics::models::hanwha
{

/**
 * @brief Hanwha HCR-12A의 FK 및 제어 제한 참조 상수.
 * @details
 * 출처: 관절 pivot은 STEP 체결 형상에서 읽은 CAD 값(mm)을 1000으로 나눈 robot-base bind pose [m];
 * 축은 controller-ready GLB의 Joint-local 단위축이며 이 GLB에서 moving node의 bind rotation은 identity다.
 * 위치 제한과 최대 속도는 Hanwha HCR-A (2G) 제조사 값을 각각 deg→rad, deg/s→rad/s로 바꿨다.
 * 이는 FK와 command 제한의 모델 참조값이며, 모든 HCR 제품 변형에 공통인 값이라고 가정하지 않는다.
 * 각 원소: 이름, base bind pivot [m], local axis, 최소·최대각 [rad], 최대 각속도 [rad/s].
 */
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

/**
 * @brief 이 controller-ready GLB의 bind pose에서 측정한 ToolFrame robot-base 위치 [m].
 * @details 모델 원점 기준 FK 참조점이다. Scene에 배치된 로봇 root의 world 변환은 포함하지 않는다.
 */
inline constexpr Vec3 kHcr12aToolFrameBindWorldMeters{0.0, 1.0150, 0.9145};

/**
 * @brief HCR-12A 모델 식별자, 관절 순서, Link 대응과 ToolFrame 변환.
 * @details
 * ToolFrame은 J6에서 고정된 frame이며 공구 장착면의 참조다. 말단에 장착하는 공구의 TCP pose는 여기에 포함되지 않는다.
 * 내부 배열은 정적 상수 배열을 가리키므로 RobotSpecification view가 사용되는 동안 유효하다.
 */
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
