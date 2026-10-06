#pragma once

#include "robotics/models/ModelTypes.h"

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace grasplink::robotics::models
{

/**
 * @brief Gripper의 회전 관절 하나에 이름, 중심, 축, 움직임 관계와 허용각을 지정한다.
 * @details bindPivotMeters는 초기 모델 자세에서 Gripper root를 기준으로 측정한 회전 중심 [m]이다.
 * axis는 해당 관절 Local 좌표계에서 본 회전축 방향이며 단위는 없다.
 * actuatorMaster가 true인 관절은 움직임을 정하는 기준(master) 각도를 제공한다.
 * mimic 관절은 기준 각도에 masterMultiplier를 곱한 값으로 함께 회전한다.
 * 이 고정 비율은 접촉 뒤 손가락이 물체 모양에 적응하는 움직임을 뜻하지 않는다.
 */
struct GripperJointSpecification
{
    /// GLB node와 Flecs Entity에서 이 관절을 찾는 이름이다. 두 곳의 이름이 일치해야 한다.
    std::string_view name;

    /// Gripper의 모델 초기 자세(bind pose)에서 root를 기준으로 측정한 관절 회전 중심 위치 [m]다.
    Vec3 bindPivotMeters;

    /// 관절 자체의 Local 좌표계에서 회전하는 축 방향이다. 방향을 나타내므로 단위가 없다.
    Axis3 axis;

    /// 기준(master) 관절 각에 곱해 자유공간에서 이 관절의 각도를 정하는 배율이다.
    double masterMultiplier = 1.0;

    /// 이 linkage 관절에 적용할 최소·최대 각도 [rad]다.
    double minPositionRadians = 0.0;
    double maxPositionRadians = 0.0;

    /// true면 자유공간 계산의 기준(master) 관절이다. 이 표식은 독립 모터가 실제로 몇 개인지를 나타내지 않는다.
    bool actuatorMaster = false;
};

/**
 * @brief Gripper 요청 code 범위와 관절 목록, 닫힘 기준각을 제공한다.
 * @details 문자열과 관절 배열은 원본을 빌려 쓰므로 Controller와 계산기보다 오래 살아야 한다.
 * 위치·속도·힘 code 0..255는 장치 통신에 쓰는 정수값이며 mm, mm/s, N 단위가 아니다.
 * nominalMasterClosedRadians는 물체와 접촉하지 않는 상태에서 기준 관절이 닫힐 때의 각도 [rad]다.
 * 이 값은 장치 모터 축 자체의 회전각이 아니다.
 * 사양은 접촉한 물체 모양에 맞춰 손가락이 따로 움직이는 기능도 정의하지 않는다.
 */
struct GripperSpecification
{
    /// UI나 로그에서 보여 줄 제조사 이름이다.
    std::string_view manufacturer;

    /// 장치 또는 로봇 제품을 식별하는 모델명이다.
    std::string_view model;

    /// 장치에 보내는 위치 요청 code의 범위다. 거리나 각도가 아니라 단위 없는 raw code다.
    std::uint8_t positionRequestMin = 0;
    std::uint8_t positionRequestMax = 255;

    /// 장치에 보내는 속도 요청 code의 범위다. 실제 속도 [mm/s]가 아니라 단위 없는 raw code다.
    std::uint8_t speedRequestMin = 0;
    std::uint8_t speedRequestMax = 255;

    /// 장치에 보내는 힘 요청 code의 범위다. 실제 힘 [N]이 아니라 단위 없는 raw code다.
    std::uint8_t forceRequestMin = 0;
    std::uint8_t forceRequestMax = 255;

    /// 자유공간에서 공칭 닫힘 상태를 나타내는 기준 linkage 관절 각 [rad]다. 실제 모터 축의 각도로 해석하면 안 된다.
    double nominalMasterClosedRadians = 0.0;

    /// 자유공간 관절 계산에 쓰는 배열이다. 포인터만 저장하므로 배열 원본은 이 사양을 쓰는 동안 유효해야 한다.
    const GripperJointSpecification* joints = nullptr;

    /// joints가 가리키는 관절 사양 배열의 원소 개수다.
    std::size_t jointCount = 0;
};

} // namespace grasplink::robotics::models
