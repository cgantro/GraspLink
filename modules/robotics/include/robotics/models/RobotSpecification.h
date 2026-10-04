#pragma once

#include "robotics/models/ModelTypes.h"

#include <cstddef>
#include <string_view>

namespace grasplink::robotics::models
{

/** @brief 특정 robot model의 하나의 revolute joint 상수. */
struct JointSpecification
{
    /*
     * [추가 용어 설명]
     * - Revolute Joint: 직선 이동이 아니라 한 축을 중심으로 회전하는 관절.
     * - Bind pose: GLB/CAD를 처음 불러온 기준 자세. 관절 pivot/축을 정의하는 기준 자세다.
     * - Pivot: 관절이 실제로 회전하는 중심점.
     * - Joint local axis: 해당 관절 자신을 기준으로 본 회전축 방향.
     * - Joint limit: 관절이 허용되는 최소/최대 각도 범위.
     */

    // GLB/Flecs에서 대응 Joint를 찾을 때 사용하는 논리 이름. 예: "J1".
    std::string_view name;

    // Bind pose에서의 관절 회전 중심 [m]. 현재 HCR 데이터는 controller-ready asset/CAD 기준값이다.
    Vec3 bindPivotMeters;

    // 해당 Joint의 local frame 기준 회전축. 방향값이므로 물리 단위는 없다.
    Axis3 axis;

    // 허용 최소 절대 관절각 [rad].
    double minPositionRadians = 0.0;

    // 허용 최대 절대 관절각 [rad].
    double maxPositionRadians = 0.0;

    // 이 Joint가 허용하는 최대 각속도 크기 [rad/s].
    double maxVelocityRadiansPerSecond = 0.0;
};

/**
 * @brief robot model별 상수를 가리키는 non-owning specification view.
 * @note joints는 프로그램 수명 동안 유효한 constexpr/static 배열을 가리켜야 한다.
 */
struct RobotSpecification
{
    /*
     * [추가 용어 설명]
     * non-owning view는 관절 배열을 복사하거나 소유하지 않고 주소만 가리킨다는 뜻이다.
     * 그래서 joints가 가리키는 배열이 RobotSpecification보다 먼저 사라지면 안 된다.
     */

    // 제조사 표시 이름. 예: "Hanwha Robotics".
    std::string_view manufacturer;

    // 제품 모델명. 예: "HCR-12A".
    std::string_view model;

    // J1..Jn 순서로 저장된 JointSpecification 배열의 첫 원소 주소.
    const JointSpecification* joints = nullptr;

    // joints 배열 원소 개수. RobotState/JointMoveCommand의 JointVector 길이 기준이 된다.
    std::size_t jointCount = 0;
};

} // namespace grasplink::robotics::models
