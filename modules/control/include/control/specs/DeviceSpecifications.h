#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

namespace control::specs
{

struct Axis3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

/** @brief Robot model별로 달라지는 하나의 revolute joint 상수. */
struct JointSpecification
{
    std::string_view name;
    Axis3 axis;
    double minPositionRadians = 0.0;
    double maxPositionRadians = 0.0;
    double maxVelocityRadiansPerSecond = 0.0;
};

/**
 * @brief 특정 로봇 모델의 non-owning specification view.
 * @note joints는 프로그램 수명 동안 유효한 constexpr/static 배열을 가리켜야 한다.
 */
struct RobotSpecification
{
    std::string_view manufacturer;
    std::string_view model;
    const JointSpecification* joints = nullptr;
    std::size_t jointCount = 0;
};

/** @brief 단일 actuator 명령에서 파생되는 gripper joint 정의. */
struct GripperJointSpecification
{
    std::string_view name;
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

} // namespace control::specs
