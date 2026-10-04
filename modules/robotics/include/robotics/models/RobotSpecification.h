#pragma once

#include "robotics/models/ModelTypes.h"

#include <cstddef>
#include <string_view>

namespace grasplink::robotics::models
{

/** @brief 특정 robot model의 하나의 revolute joint 상수. */
struct JointSpecification
{
    std::string_view name;
    Vec3 bindPivotMeters;
    Axis3 axis;
    double minPositionRadians = 0.0;
    double maxPositionRadians = 0.0;
    double maxVelocityRadiansPerSecond = 0.0;
};

/**
 * @brief robot model별 상수를 가리키는 non-owning specification view.
 * @note joints는 프로그램 수명 동안 유효한 constexpr/static 배열을 가리켜야 한다.
 */
struct RobotSpecification
{
    std::string_view manufacturer;
    std::string_view model;
    const JointSpecification* joints = nullptr;
    std::size_t jointCount = 0;
};

} // namespace grasplink::robotics::models
