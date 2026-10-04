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
    std::string_view name;
    Vec3 bindPivotMeters;
    Axis3 axis;
    double masterMultiplier = 1.0;
    double minPositionRadians = 0.0;
    double maxPositionRadians = 0.0;
    bool actuatorMaster = false;
};

/** @brief 특정 gripper model의 command 범위와 linkage 상수. */
struct GripperSpecification
{
    std::string_view manufacturer;
    std::string_view model;

    std::uint8_t positionRequestMin = 0;
    std::uint8_t positionRequestMax = 255;
    std::uint8_t speedRequestMin = 0;
    std::uint8_t speedRequestMax = 255;
    std::uint8_t forceRequestMin = 0;
    std::uint8_t forceRequestMax = 255;

    double nominalMasterClosedRadians = 0.0;
    const GripperJointSpecification* joints = nullptr;
    std::size_t jointCount = 0;
};

} // namespace grasplink::robotics::models
