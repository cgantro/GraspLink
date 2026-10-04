#pragma once

#include "robotics/models/RobotSpecification.h"

#include <array>

namespace grasplink::robotics::models::hanwha
{

/**
 * @file Hcr12a.h
 * @brief Hanwha HCR-12A의 controller/simulation 공통 정적 규격.
 *
 * @details
 * 이 파일의 수치는 두 종류의 기준을 합쳐 runtime 단위로 정규화한 값이다.
 *
 * 1) 관절 pivot/axis
 *    - HCR-12A CAD/STEP의 실제 체결 원통 중심을 기준으로 얻은 회전 중심을 meter로 변환했다.
 *    - 예: J1의 CAD 중심 y=198.5 mm -> 0.1985 m.
 *    - controller-ready GLB에서는 J1~J6 Joint Node bind rotation을 identity로 만들고,
 *      실제 회전축을 별도 axis로 정의했다.
 *
 * 2) 관절 angle/velocity limit
 *    - 제품/프로젝트 simulation spec의 degree, degree/s 값을 robotics 공통 단위인 rad, rad/s로 변환했다.
 *    - 변환식: radians = degrees * pi / 180.
 *
 * Graphics 적용 경로는 다음과 같다.
 *
 * `HCR-12A specification axis + RobotState q[rad]`
 *   -> RobotTransformAdapter의 `glm::angleAxis(q, axis)`
 *   -> quaternion을 현재 ECS Rotation 형식(Euler rad)으로 변환
 *   -> TransformSystem에서 `T * R * S` local matrix 생성
 *   -> `ParentWorld * Local`로 최종 GLB link world matrix 계산
 *
 * 관절 pivot translation은 GLB hierarchy에 이미 반영되어 있다. 따라서 RobotTransformAdapter는
 * `bindPivotMeters`를 매 frame 다시 더하지 않고 회전만 Joint Entity에 적용한다.
 */

/**
 * @brief HCR-12A J1~J6 규격. 배열 순서는 Controller의 JointVector 순서와 동일하다.
 *
 * 각 항목 순서:
 * `{name, bindPivot[m], localAxis, minAngle[rad], maxAngle[rad], maxVelocity[rad/s]}`
 */
inline constexpr std::array<JointSpecification, 6> kHcr12aJoints{{
    // J1: base yaw. Pivot=(0,198.5,0) mm, axis=+Y, range=-180..180 deg, max=130 deg/s.
    {"J1", { 0.0000, 0.1985, 0.00000}, {0.0, 1.0, 0.0}, -3.141593,  3.141593, 2.268928},

    // J2: shoulder pitch. Pivot=(153.5,310,100) mm, axis=+X, range=-165..135 deg, max=130 deg/s.
    {"J2", { 0.1535, 0.3100, 0.10000}, {1.0, 0.0, 0.0}, -2.879793,  2.356194, 2.268928},

    // J3: elbow pitch. Pivot=(109.5,910,100) mm, axis=+X, range=-85..245 deg, max=200 deg/s.
    {"J3", { 0.1095, 0.9100, 0.10000}, {1.0, 0.0, 0.0}, -1.483530,  4.276057, 3.490659},

    // J4: wrist roll. Pivot=(0,1015,233.5) mm, axis=+Z, range=-190..190 deg, max=200 deg/s.
    {"J4", { 0.0000, 1.0150, 0.23350}, {0.0, 0.0, 1.0}, -3.316126,  3.316126, 3.490659},

    // J5: wrist pitch. Pivot=(-138.5,1015,691) mm, axis=+X, range=-170..170 deg, max=200 deg/s.
    {"J5", {-0.1385, 1.0150, 0.69100}, {1.0, 0.0, 0.0}, -2.967060,  2.967060, 3.490659},

    // J6: tool roll. Pivot=(0,1015,854.75) mm, axis=+Z, range=-360..360 deg, max=200 deg/s.
    {"J6", { 0.0000, 1.0150, 0.85475}, {0.0, 0.0, 1.0}, -6.283185,  6.283185, 3.490659},
}};

/**
 * @brief Bind pose에서 HCR-12A ToolFrame의 world 위치 [m].
 * @note 현재 값은 `(0, 1.0150, 0.9145)`이며 향후 FK 검증 시 계산된 TCP와 비교할 기준점이다.
 */
inline constexpr Vec3 kHcr12aToolFrameBindWorldMeters{0.0, 1.0150, 0.9145};

/** @brief HCR-12A 모델 전체를 가리키는 non-owning RobotSpecification view. */
inline constexpr RobotSpecification kHcr12a{
    "Hanwha Robotics",
    "HCR-12A",
    kHcr12aJoints.data(),
    kHcr12aJoints.size()};

} // namespace grasplink::robotics::models::hanwha
