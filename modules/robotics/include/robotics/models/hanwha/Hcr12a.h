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
    // J1은 로봇 base를 좌우로 돌리는 첫 번째 회전 관절이다.
    {"J1", { 0.0000, 0.1985, 0.00000}, {0.0, 1.0, 0.0}, -3.141593,  3.141593, 2.268928},

    // J2는 어깨 위치에서 팔을 위아래로 움직이는 관절이다.
    {"J2", { 0.1535, 0.3100, 0.10000}, {1.0, 0.0, 0.0}, -2.879793,  2.356194, 2.268928},

    // J3는 팔꿈치처럼 팔 중간을 접고 펴는 관절이다.
    {"J3", { 0.1095, 0.9100, 0.10000}, {1.0, 0.0, 0.0}, -1.483530,  4.276057, 3.490659},

    // J4는 손목 축을 따라 공구 쪽을 비트는 관절이다.
    {"J4", { 0.0000, 1.0150, 0.23350}, {0.0, 0.0, 1.0}, -3.316126,  3.316126, 3.490659},

    // J5는 손목을 위아래로 꺾는 관절이다.
    {"J5", {-0.1385, 1.0150, 0.69100}, {1.0, 0.0, 0.0}, -2.967060,  2.967060, 3.490659},

    // J6는 손목 끝에서 공구 방향을 비트는 마지막 관절이다.
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
 * @brief 이 GLB 초기 자세에서 ToolFrame까지의 위치 [m]를 Robot base 좌표로 나타낸다. ToolFrame 위치는 장착 공구의 실제 TCP 위치와 다를 수 있다.
 * @details 모델 원점 기준 FK 참조점이다. Scene에 배치된 로봇 root의 world 변환은 포함하지 않는다.
 */
inline constexpr Vec3 kHcr12aToolFrameBindWorldMeters{0.0, 1.0150, 0.9145};

/**
 * @brief HCR-12A 이름, 관절 배열 순서, Link를 움직이는 관절 및 마지막 관절에서 ToolFrame까지의 변환을 제공한다.
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
