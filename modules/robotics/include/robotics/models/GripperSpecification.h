#pragma once

#include "robotics/models/ModelTypes.h"

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace grasplink::robotics::models
{

/**
 * @brief 자유공간 gripper linkage의 회전 관절 상수.
 * @details pivot은 specification이 정한 Gripper root bind pose 위치 [m]이고 axis는 joint-local 방향이다.
 * masterMultiplier는 master angle에서 다른 관절 각도를 정하는 고정 mimic 비율이다. 접촉 후 under-actuated 적응 동작은 나타내지 않는다.
 */
struct GripperJointSpecification
{
    /// GLB/Flecs에서 대응하는 Joint Node 이름.
    std::string_view name;

    /// Gripper root bind pose 기준 회전 중심 [m].
    Vec3 bindPivotMeters;

    /// joint-local frame의 회전축 방향. 단위 없음.
    Axis3 axis;

    /// 자유공간 mimic 계산에서 master angle에 곱하는 계수.
    double masterMultiplier = 1.0;

    /// linkage 관절 허용 범위 [rad].
    double minPositionRadians = 0.0;
    double maxPositionRadians = 0.0;

    /// 자유공간 mimic 계산의 기준 관절 여부. 독립 모터 수를 뜻하지 않는다.
    bool actuatorMaster = false;
};

/**
 * @brief Gripper command 범위와 자유공간 linkage 참조를 묶는다.
 * @details 배열과 문자열은 소유하지 않으므로 사용하는 동안 대상이 유효해야 한다. request의 0..255는 장치 프로토콜 값이며
 * 물리 위치·속도·힘 단위가 아니다. nominal closed 값은 linkage master 관절각이지 장치 모터축 각도가 아니다.
 * 접촉 후 under-actuated grasp 적응은 이 specification에 포함되지 않는다.
 */
struct GripperSpecification
{
    /// 제조사 표시 이름.
    std::string_view manufacturer;

    /// 제품 모델명.
    std::string_view model;

    /// 위치 요청 프로토콜 범위. 물리 거리나 각도가 아닌 raw 값.
    std::uint8_t positionRequestMin = 0;
    std::uint8_t positionRequestMax = 255;

    /// 속도 요청 프로토콜 범위. 실제 [mm/s]가 아닌 raw 값.
    std::uint8_t speedRequestMin = 0;
    std::uint8_t speedRequestMax = 255;

    /// 힘 요청 프로토콜 범위. 실제 [N]가 아닌 raw 값.
    std::uint8_t forceRequestMin = 0;
    std::uint8_t forceRequestMax = 255;

    /// 자유공간 nominal closed pose의 master linkage 기준각 [rad]. 모터축 각도가 아니다.
    double nominalMasterClosedRadians = 0.0;

    /// 자유공간 linkage 관절 배열. 대상은 이 view보다 오래 살아야 한다.
    const GripperJointSpecification* joints = nullptr;

    /// joints 배열 원소 수.
    std::size_t jointCount = 0;
};

} // namespace grasplink::robotics::models
