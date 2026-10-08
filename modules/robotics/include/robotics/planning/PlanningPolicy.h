#pragma once

#include <cstddef>

namespace grasplink::robotics::planning
{

/**
 * @brief 선형 TCP 경로를 계산할 때 정확도와 계산량을 조절한다.
 * @details 각 간격은 관절 충돌 검사나 TCP 경로 계산에서 사이의 점을 얼마나 촘촘히 검사할지 정한다.
 * 간격이 작을수록 더 많은 IK와 충돌 검사를 수행하며, 전체 경로 구간 수 제한은 계산량이 과도하게 커지는 것을 막는다.
 */
struct PlanningPolicy final
{
    double jointCollisionSampleSpacingRadians = 0.08;
    double linearPositionSampleSpacingMeters = 0.01;
    double linearOrientationSampleSpacingRadians = 0.05;
    std::size_t maximumPathIntervals = 4096;
};

inline constexpr PlanningPolicy kDefaultPlanningPolicy{};

} // namespace grasplink::robotics::planning
