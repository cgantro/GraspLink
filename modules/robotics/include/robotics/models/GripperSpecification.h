#pragma once

#include "robotics/models/ModelTypes.h"

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace grasplink::robotics::models
{

/** @brief 단일 actuator command에서 파생되는 gripper linkage joint 정의. */
struct GripperJointSpecification
{
    /*
     * [추가 용어 설명]
     * - Actuator(구동기): 실제 움직임을 만드는 모터/감속기 같은 구동원.
     * - Link(링크): 관절과 관절 사이의 단단한 부품.
     * - Linkage(링키지): 여러 Link와 Joint가 기계적으로 연결되어 함께 움직이는 구조.
     * - Master joint: 여러 관절 움직임을 계산할 때 기준으로 삼는 대표 관절.
     * - Mimic joint: 독립 명령을 받지 않고 master joint의 움직임을 일정 비율/부호로 따라가는 관절.
     * - Pivot: 관절이 회전하는 중심점.
     * - Free-space: 손가락이 물체와 접촉하지 않고 공중에서 열리고 닫히는 상태.
     * - Under-actuated: 관절 수보다 구동기 수가 적어 접촉 후 일부 관절이 수동적으로 적응하는 구조.
     *
     * 자유공간에서 각 관절 목표각은 기본적으로 다음 관계로 계산한다.
     * jointAngle = masterAngle * masterMultiplier
     */

    // GLB/Flecs에서 대응되는 linkage Joint Node 이름.
    std::string_view name;

    // Gripper root의 bind pose를 기준으로 한 회전 중심 [m]. Mesh 중심이 아니라 관절 hinge 중심이다.
    Vec3 bindPivotMeters;

    // 해당 Joint local frame에서의 회전축. 방향값이므로 단위는 없다.
    Axis3 axis;

    // master angle q에 곱하는 계수. +1은 같은 방향, -1은 반대 방향, 0.5라면 절반 각도다.
    double masterMultiplier = 1.0;

    // 이 linkage joint의 허용 최소 각도 [rad].
    double minPositionRadians = 0.0;

    // 이 linkage joint의 허용 최대 각도 [rad].
    double maxPositionRadians = 0.0;

    // true이면 자유공간 기구학 계산에서 대표 master joint로 취급한다.
    // 실제 제품에 독립 모터가 하나 더 있다는 뜻이 아니라 Simulation 계산 기준을 표시한다.
    bool actuatorMaster = false;
};

/** @brief 특정 gripper model의 command 범위와 linkage 상수. */
struct GripperSpecification
{
    // 제조사 표시 이름. 예: "Robotiq".
    std::string_view manufacturer;

    // 제품 모델명. 예: "2F-85".
    std::string_view model;

    // 위치 command의 최소/최대 raw 값. mm 또는 rad가 아니라 protocol 정수 영역이다.
    std::uint8_t positionRequestMin = 0;
    std::uint8_t positionRequestMax = 255;

    // 속도 command의 최소/최대 raw 값. 실제 mm/s 자체가 아니다.
    std::uint8_t speedRequestMin = 0;
    std::uint8_t speedRequestMax = 255;

    // 힘 command의 최소/최대 raw 값. 실제 Newton[N] 자체가 아니다.
    std::uint8_t forceRequestMin = 0;
    std::uint8_t forceRequestMax = 255;

    // Free-space 완전 닫힘 자세에서 사용하는 master linkage 기준각 [rad].
    // 실제 내부 motor shaft angle이 아니라 Simulation의 대표 joint angle이다.
    double nominalMasterClosedRadians = 0.0;

    // GripperJointSpecification 배열의 첫 원소 주소. 이 구조체가 배열 메모리를 소유하지 않는다.
    const GripperJointSpecification* joints = nullptr;

    // joints 배열 원소 개수. 즉 Simulation에서 정의한 linkage joint 수다.
    std::size_t jointCount = 0;
};

} // namespace grasplink::robotics::models
