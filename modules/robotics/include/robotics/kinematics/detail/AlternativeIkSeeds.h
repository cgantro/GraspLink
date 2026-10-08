#pragma once

#include "robotics/core/ControlTypes.h"
#include "robotics/models/RobotSpecification.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace grasplink::robotics::kinematics::detail
{
inline double HaltonValue(std::size_t index, std::size_t base)
{
    double value = 0.0;
    double scale = 1.0;
    while (index > 0)
    {
        scale /= static_cast<double>(base);
        value += scale * static_cast<double>(index % base);
        index /= base;
    }
    return value;
}

inline void BuildIkRestartSeed(std::size_t sample, const JointVector& start,
    const models::RobotSpecification& specification, JointVector& seed)
{
    constexpr std::array<std::size_t, 6> primeBases{2, 3, 5, 7, 11, 13};
    for (std::size_t jointIndex = 0; jointIndex < seed.size(); ++jointIndex)
    {
        const auto& joint = specification.joints[jointIndex];
        const double range = joint.maxPositionRadians - joint.minPositionRadians;
        if (range <= 1e-12)
        {
            seed[jointIndex] = start[jointIndex];
            continue;
        }
        const double startFraction = (start[jointIndex] - joint.minPositionRadians) / range;
        const double shiftedFraction = HaltonValue(sample,
            primeBases[jointIndex % primeBases.size()]) + startFraction - 0.5;
        seed[jointIndex] = joint.minPositionRadians +
            (shiftedFraction - std::floor(shiftedFraction)) * range;
    }
}

inline void AppendIkRestartSeeds(std::vector<JointVector>& seeds,
    const JointVector& start, const models::RobotSpecification& specification,
    std::size_t maximumCount)
{
    const std::size_t maximumSamples = maximumCount * 8;
    for (std::size_t sample = 1; seeds.size() < maximumCount && sample <= maximumSamples; ++sample)
    {
        JointVector candidate(start.size());
        BuildIkRestartSeed(sample, start, specification, candidate);
        const bool duplicate = std::any_of(seeds.begin(), seeds.end(), [&](const JointVector& existing)
        {
            for (std::size_t joint = 0; joint < candidate.size(); ++joint)
            {
                if (std::abs(existing[joint] - candidate[joint]) > 1e-8)
                    return false;
            }
            return true;
        });
        if (!duplicate)
            seeds.push_back(std::move(candidate));
    }
}

/**
 * @brief 현재 관절각에서 J1·J3·J5를 반사한 대체 IK 시작각을 만든다.
 * @details 관절 허용 범위의 중간을 기준으로 반사한 여덟 분기 조합 중 현재 각도와 같은 조합은 제외한다.
 * 조합 순서는 J1, J3, J5 비트 순서로 고정하고 현재 각도와 중복 후보는 처음 나온 것만 남긴다.
 * 이 제한된 후보군은 대표적인 팔 자세 분기를 시험하기 위한 것이며 가능한 IK 해 전체를 열거하지 않는다.
 */
inline std::vector<JointVector> BuildAlternativeIkSeeds(
    const JointVector& start, const models::RobotSpecification& specification)
{
    constexpr std::array<std::size_t, 3> branchJoints{0, 2, 4};
    std::vector<JointVector> seeds;
    const auto addSeed = [&](JointVector candidate)
    {
        for (const auto& existing : seeds)
        {
            bool same = true;
            for (std::size_t joint = 0; joint < candidate.size(); ++joint)
                same = same && std::abs(existing[joint] - candidate[joint]) <= 1e-8;
            if (same)
                return;
        }
        seeds.push_back(std::move(candidate));
    };

    for (unsigned mask = 1; mask < (1U << branchJoints.size()); ++mask)
    {
        JointVector candidate = start;
        bool changed = false;
        for (std::size_t bit = 0; bit < branchJoints.size(); ++bit)
        {
            const std::size_t jointIndex = branchJoints[bit];
            if ((mask & (1U << bit)) == 0 || jointIndex >= candidate.size())
                continue;

            const auto& joint = specification.joints[jointIndex];
            candidate[jointIndex] = std::clamp(
                joint.minPositionRadians + joint.maxPositionRadians - candidate[jointIndex],
                joint.minPositionRadians,
                joint.maxPositionRadians);
            changed = changed || std::abs(candidate[jointIndex] - start[jointIndex]) > 1e-8;
        }
        if (!changed)
            continue;
        addSeed(std::move(candidate));
    }
    return seeds;
}
}
