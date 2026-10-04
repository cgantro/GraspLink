#pragma once

#include "robotics/models/ModelTypes.h"

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace grasplink::robotics::models
{

/**
 * @brief 하나의 actuator/master command에서 파생되는 gripper linkage joint 정적 규격.
 *
 * @details
 * Robotiq 2F-85처럼 하나의 actuator가 여러 knuckle/tip joint를 동시에 움직이는 경우,
 * master joint angle q에 `masterMultiplier`를 곱해 각 joint의 free-space 목표각을 계산할 수 있다.
 * 접촉 이후 under-actuated 적응 동작은 이 정적 mimic 관계만으로 표현하지 않고 Physics 계층에서 처리한다.
 */
struct GripperJointSpecification
{
    /** @brief GLB/Flecs에서 해당 linkage Joint Node를 찾기 위한 이름. */
    std::string_view name;

    /** @brief Gripper root bind frame 기준 joint pivot [x,y,z], 단위 meter [m]. */
    Vec3 bindPivotMeters;

    /** @brief Joint local frame 기준 회전축. 무차원 unit direction. */
    Axis3 axis;

    /**
     * @brief Master angle q에 곱하는 mimic 계수.
     * 예: +1이면 q와 같은 방향/크기, -1이면 반대 방향/같은 크기로 회전한다.
     */
    double masterMultiplier = 1.0;

    /** @brief 이 linkage joint의 허용 최소 각도 [rad]. */
    double minPositionRadians = 0.0;

    /** @brief 이 linkage joint의 허용 최대 각도 [rad]. */
    double maxPositionRadians = 0.0;

    /** @brief true이면 이 joint가 논리적 master actuator joint임을 표시한다. */
    bool actuatorMaster = false;
};

/**
 * @brief 특정 gripper model의 command 범위와 linkage 정적 규격을 묶는 non-owning view.
 *
 * @details
 * command 범위는 물리 단위가 아니라 장치가 노출하는 normalized/raw request domain이다.
 * Robotiq 2F-85의 경우 position/speed/force 모두 0..255이며, position 0=open, 255=closed다.
 */
struct GripperSpecification
{
    /** @brief 제조사 표시 이름. */
    std::string_view manufacturer;

    /** @brief 모델 표시 이름. */
    std::string_view model;

    /** @brief Position request 최소 raw 값. */
    std::uint8_t positionRequestMin = 0;

    /** @brief Position request 최대 raw 값. */
    std::uint8_t positionRequestMax = 255;

    /** @brief Speed request 최소 raw 값. */
    std::uint8_t speedRequestMin = 0;

    /** @brief Speed request 최대 raw 값. */
    std::uint8_t speedRequestMax = 255;

    /** @brief Force request 최소 raw 값. */
    std::uint8_t forceRequestMin = 0;

    /** @brief Force request 최대 raw 값. */
    std::uint8_t forceRequestMax = 255;

    /**
     * @brief Free-space 완전 닫힘 자세에서 master linkage joint의 기준 각도 [rad].
     * @note 실제 motor shaft angle이 아니라 simulation kinematic master joint angle이다.
     */
    double nominalMasterClosedRadians = 0.0;

    /** @brief Linkage joint 규격 배열 시작 주소. 소유권은 없다. */
    const GripperJointSpecification* joints = nullptr;

    /** @brief joints 배열 원소 개수. */
    std::size_t jointCount = 0;
};

} // namespace grasplink::robotics::models
