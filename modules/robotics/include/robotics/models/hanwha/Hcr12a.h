#pragma once

#include "robotics/models/RobotSpecification.h"

#include <array>

namespace grasplink::robotics::models::hanwha
{

/**
 * @file Hcr12a.h
 * @brief Hanwha HCR-12A의 고정된 관절 사양을 Controller/Simulation이 함께 사용하도록 모아둔 파일.
 *
 * @details
 * 이 파일은 로봇을 움직이는 알고리즘이 아니라 "HCR-12A라는 로봇은 어떤 관절 구조와 제한을 갖는가"를 정의한다.
 *
 * 로보틱스 용어:
 * - J1~J6: base에서 tool 방향으로 번호를 붙인 6개의 회전관절.
 * - Base: 로봇이 바닥/설치면에 고정되는 시작 부분.
 * - Shoulder: 사람의 어깨처럼 팔의 큰 상부 회전을 만드는 관절 영역.
 * - Elbow: 사람의 팔꿈치처럼 중간 팔을 꺾는 관절 영역.
 * - Wrist: 말단 공구 가까이에서 자세를 조절하는 관절 영역.
 * - Yaw: 세로축 주변으로 좌우 방향을 바꾸는 회전.
 * - Pitch: 앞뒤로 숙이거나 드는 형태의 회전.
 * - Roll: 길이축 주변으로 비트는 형태의 회전.
 * - Pivot: 실제 관절이 회전하는 중심점.
 * - Axis: 그 Pivot을 지나며 회전의 중심이 되는 방향.
 * - Joint limit: 관절이 허용되는 최소/최대 각도 범위.
 * - Max velocity: 관절이 허용되는 최대 회전속도.
 *
 * 데이터 출처/변환:
 * 1) Pivot/axis
 *    HCR-12A STEP/CAD의 실제 체결 원통 중심과 축을 기준으로 얻었고 mm -> m로 변환했다.
 * 2) Angle/velocity limit
 *    degree, degree/s 값을 공통 runtime 단위인 rad, rad/s로 변환했다.
 *
 * 변환식:
 * `radian = degree * pi / 180`
 *
 * 현재 controller-ready GLB에서는 J1~J6 Joint Node의 bind rotation이 identity다.
 * 따라서 Controller가 계산한 q[rad]와 아래 axis를 이용해 Joint 회전을 직접 만들 수 있다.
 */

/**
 * @brief HCR-12A의 J1~J6 고정 사양.
 *
 * @details
 * 배열 순서는 항상 `[J1,J2,J3,J4,J5,J6]`이며 RobotState/JointMoveCommand도 같은 순서를 사용한다.
 * 각 원소 형식은 다음과 같다.
 *
 * `{name, bindPivot[m], localAxis, minAngle[rad], maxAngle[rad], maxVelocity[rad/s]}`
 */
inline constexpr std::array<JointSpecification, 6> kHcr12aJoints{{
    // J1: base 전체를 좌우로 돌리는 yaw 관절.
    // Pivot=(0,198.5,0) mm -> (0,0.1985,0) m, local +Y축, -180~180 deg, 최대 130 deg/s.
    {"J1", { 0.0000, 0.1985, 0.00000}, {0.0, 1.0, 0.0}, -3.141593,  3.141593, 2.268928},

    // J2: shoulder pitch 관절. 첫 번째 큰 팔 구간을 앞/뒤 방향으로 들어 올리거나 내리는 역할.
    // Pivot=(153.5,310,100) mm, local +X축, -165~135 deg, 최대 130 deg/s.
    {"J2", { 0.1535, 0.3100, 0.10000}, {1.0, 0.0, 0.0}, -2.879793,  2.356194, 2.268928},

    // J3: elbow pitch 관절. 중간 팔 구간을 접거나 펴는 역할.
    // Pivot=(109.5,910,100) mm, local +X축, -85~245 deg, 최대 200 deg/s.
    {"J3", { 0.1095, 0.9100, 0.10000}, {1.0, 0.0, 0.0}, -1.483530,  4.276057, 3.490659},

    // J4: wrist roll 계열 관절. 손목부를 local +Z축 중심으로 비트는 역할.
    // Pivot=(0,1015,233.5) mm, -190~190 deg, 최대 200 deg/s.
    {"J4", { 0.0000, 1.0150, 0.23350}, {0.0, 0.0, 1.0}, -3.316126,  3.316126, 3.490659},

    // J5: wrist pitch 관절. tool 방향을 위/아래로 꺾어 말단 자세를 조절하는 역할.
    // Pivot=(-138.5,1015,691) mm, local +X축, -170~170 deg, 최대 200 deg/s.
    {"J5", {-0.1385, 1.0150, 0.69100}, {1.0, 0.0, 0.0}, -2.967060,  2.967060, 3.490659},

    // J6: tool roll 관절. 최종 tool/gripper를 축 방향으로 회전시키는 마지막 관절.
    // Pivot=(0,1015,854.75) mm, local +Z축, -360~360 deg, 최대 200 deg/s.
    {"J6", { 0.0000, 1.0150, 0.85475}, {0.0, 0.0, 1.0}, -6.283185,  6.283185, 3.490659},
}};

/**
 * @brief Bind pose에서 ToolFrame의 기준 위치, world frame 기준 [m].
 *
 * @details
 * ToolFrame은 로봇 끝단에 공구/그리퍼를 붙이는 기준 좌표계다.
 * 향후 FK를 구현하면 J1~J6 각도에서 계산한 말단 위치와 이 기준점을 비교해 검증할 수 있다.
 */
inline constexpr Vec3 kHcr12aToolFrameBindWorldMeters{0.0, 1.0150, 0.9145};

/**
 * @brief 위 J1~J6 배열을 HCR-12A 하나의 RobotSpecification으로 묶은 값.
 * @details Controller는 이 값 하나를 받아 관절 개수, 이름, limit, max velocity를 공통 방식으로 사용할 수 있다.
 */
inline constexpr RobotSpecification kHcr12a{
    "Hanwha Robotics",
    "HCR-12A",
    kHcr12aJoints.data(),
    kHcr12aJoints.size()};

} // namespace grasplink::robotics::models::hanwha
