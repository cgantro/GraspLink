#pragma once

namespace grasplink::application::pick_place::config
{

inline constexpr double approachHeightMeters = 0.25;
inline constexpr double transitBoxHeightMeters = 0.45;
inline constexpr double baseExclusionRadiusMeters = 0.42;
inline constexpr double transitAnnulusRadiusMeters = 0.56;
inline constexpr double graspClearanceMeters = 0.025;
inline constexpr double linearVelocityMetersPerSecond = 2.0;
inline constexpr double angularVelocityRadiansPerSecond = 8.0;
inline constexpr double linearAccelerationMetersPerSecondSquared = 30.0;
inline constexpr double angularAccelerationRadiansPerSecondSquared = 120.0;
inline constexpr double j6UnwindToleranceRadians = 0.01745329251994329576923690768489;
inline constexpr float boxSideMeters = 0.04F;
inline constexpr float placementAreaSideMeters = boxSideMeters * 1.7320508F;
inline constexpr float placementAreaThicknessMeters = 0.002F;
inline constexpr float boxPositionHeightMeters = 0.026F;
inline constexpr float placementPositionHeightMeters = 0.006F;
inline constexpr float minimumScenarioRadiusMeters = 0.50F;
inline constexpr float maximumBoxRadiusMeters = 1.05F;
inline constexpr float maximumPlacementRadiusMeters = 1.00F;
inline constexpr float minimumBoxRepeatSeparationMeters = 0.035F;
inline constexpr float minimumPlacementRepeatSeparationMeters = 0.05F;
inline constexpr float scenarioCoordinateLimitMeters = 1.05F;
inline constexpr float scenarioRotationLimitRadians = 1.57079632679F;

}
