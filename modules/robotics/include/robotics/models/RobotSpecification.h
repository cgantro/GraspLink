#pragma once

#include "robotics/models/ModelTypes.h"

#include <cstddef>
#include <string_view>

namespace grasplink::robotics::models
{

/**
 * @brief FK와 제어 제한에 쓰는 로봇 회전 관절 상수.
 * @details bindPivotMeters는 공통 robot-base bind pose 위치다. axis는 bind pose의 joint-local 방향이며 각도는 rad,
 * 최대 속도는 rad/s다. 현재 HCR-12A 값은 제조사 한계와 CAD/GLB 기준점을 함께 사용한다.
 */
struct JointSpecification
{
    /// GLB/Flecs에서 관절을 찾는 논리 이름. 예: "J1".
    std::string_view name;

    /// bind pose의 robot-base 기준 회전 중심 [m].
    Vec3 bindPivotMeters;

    // 관절 local frame 기준 회전축 방향. 단위 없음.
    Axis3 axis;

    /// 제조사 관절 위치 제한 [rad].
    double minPositionRadians = 0.0;
    double maxPositionRadians = 0.0;

    /// 제조사 최대 각속도 [rad/s].
    double maxVelocityRadiansPerSecond = 0.0;
};

/** @brief Link 이름을 대응하는 구동 관절 배열 위치에 연결한다. */
struct LinkSpecification
{
    /// GLB/Flecs에서 식별하는 Link 이름.
    std::string_view name;
    /// joints 배열에서 이 Link를 구동하는 관절의 index.
    std::size_t jointIndex = 0;
};

/**
 * @brief 로봇 식별 정보와 정적 운동학 참조를 묶는다.
 * @details 이 구조체와 내부 string_view 및 배열 포인터는 데이터를 소유하지 않는다. 대상은 사용하는 동안 유효해야 한다.
 * joints 순서는 RobotState 및 JointMoveCommand의 관절 순서와 같아야 한다. FK는 base 기준 관절 pivot으로 Link pose를
 * 계산하고, 선택된 경우 마지막 관절 기준 ToolFrame을 별도로 계산한다. ToolFrame은 장착 공구의 TCP와 별개다.
 */
struct RobotSpecification
{

    /// 제조사 표시 이름.
    std::string_view manufacturer;

    /// 제품 모델명.
    std::string_view model;

    /// RobotState 관절 값과 같은 순서의 관절 배열.
    const JointSpecification* joints = nullptr;

    /// 상태·명령 관절 배열 길이의 기준.
    std::size_t jointCount = 0;

    /// Link와 구동 관절의 대응 배열. 대상은 이 view보다 오래 살아야 한다.
    const LinkSpecification* links = nullptr;
    /// links 배열 원소 수.
    std::size_t linkCount = 0;

    /// 마지막 관절 frame에서 ToolFrame까지의 고정 변환. translation [m].
    Pose3 toolFrameInLastJoint{};
    /// true이면 FK가 ToolFrame을 계산한다. false이면 유효 ToolFrame이 없다.
    bool hasToolFrame = false;
};

} // namespace grasplink::robotics::models
