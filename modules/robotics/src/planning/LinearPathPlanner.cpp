#include "robotics/planning/LinearPathPlanner.h"

#include "robotics/kinematics/detail/PoseMath.h"
#include "robotics/kinematics/detail/AlternativeIkSeeds.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <iterator>
#include <limits>
#include <string>
#include <utility>

namespace grasplink::robotics::planning
{
namespace
{
using models::Pose3;

constexpr double kFullTurnRadians = 2.0 * 3.14159265358979323846;
constexpr std::size_t kIkRestartSeedCount = 16;
constexpr std::size_t kMaximumIkCandidatesPerSample = 8;
constexpr std::size_t kMaximumIkSeedAttemptsPerSample = 32;
constexpr std::size_t kMaximumTcpRefinementPasses = 12;
constexpr std::size_t kMaximumPathIntervals = 4096;
constexpr double kJointLimitSearchMarginFraction = 0.1;
constexpr double kJointLimitSearchTriggerFraction = 0.05;
constexpr double kJointLimitPenaltyWeight = 0.1;
constexpr std::array<std::size_t, 6> kRestartPrimeBases{2, 3, 5, 7, 11, 13};

double HaltonValue(std::size_t index, std::size_t base)
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

void BuildRestartSeed(std::size_t sample, const JointVector& start,
    const models::RobotSpecification& specification, JointVector& seed)
{
    for (std::size_t joint = 0; joint < seed.size(); ++joint)
    {
        const auto& limits = specification.joints[joint];
        const double range = limits.maxPositionRadians - limits.minPositionRadians;
        if (range <= 1e-12)
        {
            seed[joint] = start[joint];
            continue;
        }
        const double startFraction = (start[joint] - limits.minPositionRadians) / range;
        const double shiftedFraction = HaltonValue(sample,
            kRestartPrimeBases[joint % kRestartPrimeBases.size()]) + startFraction - 0.5;
        const double fraction = shiftedFraction - std::floor(shiftedFraction);
        seed[joint] = limits.minPositionRadians +
            fraction * range;
    }
}

bool IsApproachingJ1Limit(const JointVector& start, const JointVector& target,
    const models::RobotSpecification& specification)
{
    const auto& limits = specification.joints[0];
    const double range = limits.maxPositionRadians - limits.minPositionRadians;
    if (range <= 1e-12)
        return false;
    const double startMargin = std::min(start.front() - limits.minPositionRadians,
        limits.maxPositionRadians - start.front()) / range;
    const double targetMargin = std::min(target.front() - limits.minPositionRadians,
        limits.maxPositionRadians - target.front()) / range;
    return targetMargin < kJointLimitSearchTriggerFraction && targetMargin + 1e-9 < startMargin;
}

Result AddPathSampleContext(Result failure, std::size_t sample, std::size_t sampleCount,
    const models::RobotSpecification& specification, const CartesianPose& target,
    const kinematics::IkResult* ikFailure = nullptr)
{
    std::string context = "SimRobotController: TCP path sample " + std::to_string(sample) + "/" +
        std::to_string(sampleCount) + " failed at (" +
        std::to_string(target.positionMeters[0]) + ", " +
        std::to_string(target.positionMeters[1]) + ", " +
        std::to_string(target.positionMeters[2]) + ") m";
    if (ikFailure)
    {
        if (ikFailure->status == kinematics::IkStatus::JointLimitReached)
        {
            for (std::size_t joint = 0; joint < ikFailure->jointPositionRadians.size(); ++joint)
            {
                const auto& limits = specification.joints[joint];
                const double tolerance = std::max(1e-8,
                    (limits.maxPositionRadians - limits.minPositionRadians) * 1e-8);
                const double position = ikFailure->jointPositionRadians[joint];
                if (std::abs(position - limits.minPositionRadians) <= tolerance ||
                    std::abs(position - limits.maxPositionRadians) <= tolerance)
                {
                    context += "; IK candidate reached " + std::string(limits.name) + " limit";
                    break;
                }
            }
        }
        context += "; IK residual " + std::to_string(ikFailure->positionErrorMeters) +
            " m / " + std::to_string(ikFailure->orientationErrorRadians) +
            " rad after " + std::to_string(ikFailure->iterations) + " iterations; candidate_deg=[";
        for (std::size_t joint = 0; joint < ikFailure->jointPositionRadians.size(); ++joint)
            context += (joint == 0 ? "" : ",") + std::to_string(
                ikFailure->jointPositionRadians[joint] * 360.0 / (2.0 * 3.14159265358979323846));
        context += "]";
    }
    failure.message = std::move(context) + ": " + failure.message;
    return failure;
}

struct PathCandidate
{
    JointVector joints;
    Pose3 tcpPose{};
    double score = 0.0;
    std::size_t previousCandidate = 0;
    std::size_t stableSamples = 0;
};

struct IkSeedCandidate
{
    std::size_t previousCandidate = 0;
    JointVector joints;
    bool continuation = false;
};

double JointLimitMarginFraction(const JointVector& joints,
    const models::RobotSpecification& specification)
{
    double margin = 1.0;
    for (std::size_t joint = 0; joint < joints.size(); ++joint)
    {
        const auto& limits = specification.joints[joint];
        const double range = limits.maxPositionRadians - limits.minPositionRadians;
        if (range <= 1e-12)
            continue;
        margin = std::min(margin, std::min(joints[joint] - limits.minPositionRadians,
            limits.maxPositionRadians - joints[joint]) / range);
    }
    return margin;
}

bool IsLimitRisky(const JointVector& start, const JointVector& target,
    const models::RobotSpecification& specification)
{
    const double startMargin = JointLimitMarginFraction(start, specification);
    const double targetMargin = JointLimitMarginFraction(target, specification);
    return targetMargin <= kJointLimitSearchMarginFraction ||
        (targetMargin < 0.2 && targetMargin + 1e-9 < startMargin) ||
        IsApproachingJ1Limit(start, target, specification);
}

bool IsPowerOfTwo(std::size_t value)
{
    return value != 0 && (value & (value - 1)) == 0;
}

bool IsPolicyValid(const PlanningPolicy& policy)
{
    return std::isfinite(policy.jointCollisionSampleSpacingRadians) &&
        policy.jointCollisionSampleSpacingRadians > 0.0 &&
        std::isfinite(policy.linearPositionSampleSpacingMeters) &&
        policy.linearPositionSampleSpacingMeters > 0.0 &&
        std::isfinite(policy.linearOrientationSampleSpacingRadians) &&
        policy.linearOrientationSampleSpacingRadians > 0.0 &&
        policy.maximumPathIntervals > 0 && policy.maximumPathIntervals <= kMaximumPathIntervals &&
        policy.maximumIkCandidatesPerSample > 0 &&
        policy.maximumIkCandidatesPerSample <= kMaximumIkCandidatesPerSample &&
        policy.maximumIkSeedAttemptsPerSample > 0 &&
        policy.maximumIkSeedAttemptsPerSample >= policy.maximumIkCandidatesPerSample &&
        policy.maximumIkSeedAttemptsPerSample <= kMaximumIkSeedAttemptsPerSample &&
        policy.maximumTcpRefinementPasses <= kMaximumTcpRefinementPasses &&
        std::isfinite(policy.maximumLinearTcpErrorMeters) && policy.maximumLinearTcpErrorMeters > 0.0 &&
        std::isfinite(policy.maximumAngularTcpErrorRadians) && policy.maximumAngularTcpErrorRadians > 0.0;
}

double JointTransitionCost(const JointVector& start, const JointVector& end,
    const models::RobotSpecification& specification)
{
    double score = 0.0;
    for (std::size_t joint = 0; joint < start.size(); ++joint)
    {
        const auto& limits = specification.joints[joint];
        const double range = std::max(1e-12, limits.maxPositionRadians - limits.minPositionRadians);
        const double delta = (end[joint] - start[joint]) / range;
        score += delta * delta;
        const double margin = std::min(end[joint] - limits.minPositionRadians,
            limits.maxPositionRadians - end[joint]) / range;
        const double deficit = std::max(0.0, kJointLimitSearchMarginFraction - margin) /
            kJointLimitSearchMarginFraction;
        score += kJointLimitPenaltyWeight * deficit * deficit;
    }
    return score;
}

bool SameJointCandidate(const JointVector& left, const JointVector& right)
{
    for (std::size_t joint = 0; joint < left.size(); ++joint)
    {
        if (std::abs(left[joint] - right[joint]) > 1e-6)
            return false;
    }
    return true;
}

bool JointEdgeFollowsTcpLine(const JointVector& start, const JointVector& end,
    const Pose3& startTcp, const Pose3& endTcp,
    kinematics::DampedLeastSquaresIk& inverse, const PlanningPolicy& policy)
{
    JointVector sample(start.size());
    for (const double fraction : {0.25, 0.5, 0.75})
    {
        for (std::size_t joint = 0; joint < sample.size(); ++joint)
            sample[joint] = start[joint] + (end[joint] - start[joint]) * fraction;
        const Pose3 actual = kinematics::detail::FromCartesian(inverse.EvaluateTcp(sample));
        const Pose3 expected = kinematics::detail::Interpolate(startTcp, endTcp, fraction);
        if (kinematics::detail::Length(kinematics::detail::Subtract(
                actual.positionMeters, expected.positionMeters)) > policy.maximumLinearTcpErrorMeters ||
            kinematics::detail::Length(kinematics::detail::RotationError(
                actual.rotation, expected.rotation)) > policy.maximumAngularTcpErrorRadians)
            return false;
    }
    return true;
}

std::vector<PathCandidate> BuildNextCandidates(
    std::size_t sampleIndex,
    const Pose3& targetPose,
    const std::vector<PathCandidate>& previous,
    kinematics::DampedLeastSquaresIk& inverse,
    const models::RobotSpecification& specification,
    const StateValidityChecker& stateValidityChecker,
    const kinematics::IkOptions& options,
    const PlanningPolicy& policy,
    bool& needsRefinement,
    JointStateInvalidity& invalidity,
    kinematics::IkResult& ikFailure)
{
    std::vector<IkSeedCandidate> seeds;
    seeds.reserve(std::max(policy.maximumIkSeedAttemptsPerSample, previous.size()));
    for (std::size_t parent = 0; parent < previous.size(); ++parent)
        seeds.push_back({parent, previous[parent].joints, true});

    std::vector<PathCandidate> candidates;
    candidates.reserve(policy.maximumIkCandidatesPerSample);
    std::vector<std::size_t> evaluatedParents;
    evaluatedParents.reserve(seeds.size());
    std::vector<bool> evaluatedEdgesFollowTcp;
    evaluatedEdgesFollowTcp.reserve(seeds.size());
    std::vector<double> evaluatedJointPositions;
    evaluatedJointPositions.reserve(seeds.size() * specification.jointCount);
    std::vector<bool> parentHasContinuation(previous.size(), false);
    std::vector<bool> parentNeedsSearch(previous.size(), false);
    std::vector<bool> parentHasValidChild(previous.size(), false);
    double bestFailureResidual = std::numeric_limits<double>::infinity();
    const auto evaluate = [&](const IkSeedCandidate& seed)
    {
        auto solution = inverse.SolveSingleSeed(kinematics::detail::ToCartesian(targetPose), seed.joints, options);
        if (!solution)
        {
            const double residual = solution.positionErrorMeters +
                options.orientationWeightMetersPerRadian * solution.orientationErrorRadians;
            if (residual < bestFailureResidual)
            {
                bestFailureResidual = residual;
                ikFailure = std::move(solution);
            }
            if (seed.continuation)
                parentNeedsSearch[seed.previousCandidate] = true;
            return;
        }

        const auto& parent = previous[seed.previousCandidate];
        AlignEquivalentJointAngles(solution.jointPositionRadians, parent.joints, specification);
        const double score = parent.score + JointTransitionCost(
            parent.joints, solution.jointPositionRadians, specification);
        auto duplicate = std::find_if(candidates.begin(), candidates.end(), [&](const PathCandidate& candidate)
        {
            return SameJointCandidate(candidate.joints, solution.jointPositionRadians);
        });
        if (duplicate != candidates.end() && score >= duplicate->score)
        {
            if (duplicate->previousCandidate == seed.previousCandidate)
            {
                parentHasValidChild[seed.previousCandidate] = true;
                if (seed.continuation)
                {
                    parentHasContinuation[seed.previousCandidate] = true;
                    duplicate->stableSamples = std::max(duplicate->stableSamples,
                        parent.stableSamples + 1);
                }
            }
            return;
        }
        if (candidates.size() >= policy.maximumIkCandidatesPerSample)
        {
            const auto worst = std::max_element(candidates.begin(), candidates.end(),
                [](const PathCandidate& left, const PathCandidate& right)
                {
                    return left.score < right.score;
                });
            if (score > worst->score)
                return;
        }

        bool alreadyEvaluated = false;
        for (std::size_t evaluated = 0; evaluated < evaluatedParents.size() && !alreadyEvaluated; ++evaluated)
        {
            if (evaluatedParents[evaluated] != seed.previousCandidate)
                continue;
            const auto begin = evaluatedJointPositions.begin() +
                static_cast<std::ptrdiff_t>(evaluated * specification.jointCount);
            alreadyEvaluated = std::equal(solution.jointPositionRadians.begin(),
                solution.jointPositionRadians.end(), begin);
        }
        if (alreadyEvaluated)
        {
            if (seed.continuation)
                parentHasContinuation[seed.previousCandidate] = true;
            return;
        }
        evaluatedParents.push_back(seed.previousCandidate);
        evaluatedEdgesFollowTcp.push_back(false);
        evaluatedJointPositions.insert(evaluatedJointPositions.end(),
            solution.jointPositionRadians.begin(), solution.jointPositionRadians.end());

        if (!JointEdgeFollowsTcpLine(parent.joints, solution.jointPositionRadians,
            parent.tcpPose, targetPose, inverse, policy))
        {
            needsRefinement = true;
            parentNeedsSearch[seed.previousCandidate] = true;
            if (seed.continuation)
                parentHasContinuation[seed.previousCandidate] = true;
            return;
        }
        evaluatedEdgesFollowTcp.back() = true;

        const auto pathInvalidity = ValidateJointPath(parent.joints, solution.jointPositionRadians,
            specification, stateValidityChecker, policy);
        if (pathInvalidity != JointStateInvalidity::None)
        {
            if (invalidity == JointStateInvalidity::None)
                invalidity = pathInvalidity;
            parentNeedsSearch[seed.previousCandidate] = true;
            if (seed.continuation)
                parentHasContinuation[seed.previousCandidate] = true;
            return;
        }

        parentHasValidChild[seed.previousCandidate] = true;
        const bool limitRisky = IsLimitRisky(parent.joints,
            solution.jointPositionRadians, specification);
        if (duplicate == candidates.end())
        {
            candidates.push_back({std::move(solution.jointPositionRadians), targetPose, score,
                seed.previousCandidate, limitRisky ? 0 : parent.stableSamples + 1});
            duplicate = std::prev(candidates.end());
        }
        else if (score < duplicate->score)
        {
            duplicate->joints = std::move(solution.jointPositionRadians);
            duplicate->tcpPose = targetPose;
            duplicate->score = score;
            duplicate->previousCandidate = seed.previousCandidate;
            duplicate->stableSamples = limitRisky ? 0 : parent.stableSamples + 1;
        }
        else if (seed.continuation)
            duplicate->stableSamples = std::max(duplicate->stableSamples, parent.stableSamples + 1);
        if (seed.continuation)
            parentHasContinuation[seed.previousCandidate] = true;
        if (limitRisky)
        {
            parentNeedsSearch[seed.previousCandidate] = true;
            duplicate->stableSamples = 0;
        }
        if (candidates.size() > policy.maximumIkCandidatesPerSample)
        {
            const auto worst = std::max_element(candidates.begin(), candidates.end(),
                [](const PathCandidate& left, const PathCandidate& right)
                {
                    return left.score < right.score;
                });
            candidates.erase(worst);
        }
    };

    // 모든 기존 분기의 이어짐을 먼저 검사한다. 안정된 분기에는 매 표본마다
    // 전역 seed를 더하지 않아 IK 비용을 제한하되, 그 후보 집합은 전체 seed
    // 탐색을 매번 하는 방식과 다를 수 있다. 위험 신호가 생기면 아래 탐색을 연다.
    for (const auto& seed : seeds)
        evaluate(seed);

    for (std::size_t parent = 0; parent < previous.size(); ++parent)
    {
        if (!parentHasValidChild[parent] || !parentHasContinuation[parent] ||
            IsPowerOfTwo(previous[parent].stableSamples + 1))
            parentNeedsSearch[parent] = true;
    }
    if (candidates.size() < previous.size())
        std::fill(parentNeedsSearch.begin(), parentNeedsSearch.end(), true);

    std::vector<std::vector<JointVector>> reflectedSeeds(previous.size());
    for (std::size_t parent = 0; parent < previous.size(); ++parent)
    {
        if (parentNeedsSearch[parent])
            reflectedSeeds[parent] = kinematics::detail::BuildAlternativeIkSeeds(
                previous[parent].joints, specification);
    }
    for (std::size_t branch = 0; seeds.size() < policy.maximumIkSeedAttemptsPerSample; ++branch)
    {
        bool addedSeed = false;
        for (std::size_t parent = 0; parent < previous.size() &&
            seeds.size() < policy.maximumIkSeedAttemptsPerSample; ++parent)
        {
            if (parentNeedsSearch[parent] && branch < reflectedSeeds[parent].size())
            {
                seeds.push_back({parent, reflectedSeeds[parent][branch], false});
                addedSeed = true;
            }
        }
        if (!addedSeed)
            break;
    }
    for (std::size_t restart = 1; seeds.size() < policy.maximumIkSeedAttemptsPerSample; ++restart)
    {
        bool addedSeed = false;
        for (std::size_t parent = 0; parent < previous.size() &&
            seeds.size() < policy.maximumIkSeedAttemptsPerSample; ++parent)
        {
            if (!parentNeedsSearch[parent])
                continue;
            JointVector seed(previous[parent].joints.size());
            BuildRestartSeed((sampleIndex + 1) * kIkRestartSeedCount + restart,
                previous[parent].joints, specification, seed);
            seeds.push_back({parent, std::move(seed), false});
            addedSeed = true;
        }
        if (!addedSeed)
            break;
    }
    for (std::size_t seed = previous.size(); seed < seeds.size(); ++seed)
        evaluate(seeds[seed]);

    if (candidates.empty() && needsRefinement)
    {
        JointVector sample(specification.jointCount);
        bool foundCollisionFreeEdge = false;
        JointStateInvalidity rejectedInvalidity = JointStateInvalidity::None;
        for (std::size_t evaluated = 0; evaluated < evaluatedParents.size(); ++evaluated)
        {
            if (evaluatedEdgesFollowTcp[evaluated])
                continue;
            const auto offset = evaluated * specification.jointCount;
            std::copy_n(evaluatedJointPositions.begin() + static_cast<std::ptrdiff_t>(offset),
                specification.jointCount, sample.begin());
            const auto pathInvalidity = ValidateJointPath(
                previous[evaluatedParents[evaluated]].joints, sample,
                specification, stateValidityChecker, policy);
            if (pathInvalidity == JointStateInvalidity::None)
                foundCollisionFreeEdge = true;
            else if (rejectedInvalidity == JointStateInvalidity::None)
                rejectedInvalidity = pathInvalidity;
        }
        needsRefinement = foundCollisionFreeEdge;
        invalidity = foundCollisionFreeEdge ? JointStateInvalidity::None : rejectedInvalidity;
    }
    std::sort(candidates.begin(), candidates.end(), [](const PathCandidate& left, const PathCandidate& right)
    {
        return left.score < right.score;
    });
    return candidates;
}
}

JointStateInvalidity ValidateJointPath(
    const JointVector& start,
    const JointVector& end,
    const models::RobotSpecification& specification,
    const StateValidityChecker& stateValidityChecker,
    const PlanningPolicy& policy)
{
    // 관절 하나의 변화량이 설정한 간격보다 크면 그 사이 자세도 나눠 검사한다. 시작 자세는 이미 검증됐다고 보고 끝 자세까지 확인한다.
    const auto startInvalidity = ValidateJointState(specification, start, {});
    if (startInvalidity != JointStateInvalidity::None)
        return startInvalidity;
    const auto endInvalidity = ValidateJointState(specification, end, {});
    if (endInvalidity != JointStateInvalidity::None)
        return endInvalidity;

    double maximumJointChange = 0.0;
    for (std::size_t joint = 0; joint < start.size(); ++joint)
        maximumJointChange = std::max(maximumJointChange, std::abs(end[joint] - start[joint]));
    const std::size_t intervals = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(
        maximumJointChange / policy.jointCollisionSampleSpacingRadians)));
    JointVector sample(start.size());
    for (std::size_t step = 1; step <= intervals; ++step)
    {
        const double fraction = static_cast<double>(step) / static_cast<double>(intervals);
        for (std::size_t joint = 0; joint < start.size(); ++joint)
            sample[joint] = start[joint] + (end[joint] - start[joint]) * fraction;
        // 양 끝 관절각이 유한하고 한계 안에 있으면 선형 보간한 중간 각도도 그 범위 안에 있다. 따라서 매 표본에서는 비용이 큰 충돌 검사만 수행한다.
        const auto invalidity = stateValidityChecker ?
            stateValidityChecker(sample) : JointStateInvalidity::None;
        if (invalidity != JointStateInvalidity::None)
            return invalidity;
    }
    return JointStateInvalidity::None;
}
JointStateInvalidity ValidateJointState(
    const models::RobotSpecification& specification,
    const JointVector& joints,
    const StateValidityChecker& stateValidityChecker)
{
    if (specification.joints == nullptr || joints.size() != specification.jointCount)
        return JointStateInvalidity::JointCountMismatch;
    for (std::size_t joint = 0; joint < specification.jointCount; ++joint)
    {
        if (!std::isfinite(joints[joint]))
            return JointStateInvalidity::NonFinitePosition;
        const auto& limits = specification.joints[joint];
        if (joints[joint] < limits.minPositionRadians || joints[joint] > limits.maxPositionRadians)
            return JointStateInvalidity::JointLimitViolation;
    }
    return stateValidityChecker ? stateValidityChecker(joints) : JointStateInvalidity::None;
}

Result MapJointStateInvalidity(JointStateInvalidity invalidity)
{
    switch (invalidity)
    {
    case JointStateInvalidity::None: return Result::Success();
    case JointStateInvalidity::JointCountMismatch:
    case JointStateInvalidity::NonFinitePosition:
        return {ErrorCode::InvalidCommand, "SimRobotController: invalid joint state in planned path"};
    case JointStateInvalidity::JointLimitViolation:
        return {ErrorCode::JointLimitReached, "SimRobotController: planned state exceeds a joint limit"};
    case JointStateInvalidity::EnvironmentCollision:
        return {ErrorCode::EnvironmentContact, "SimRobotController: planned state overlaps the environment"};
    case JointStateInvalidity::SelfCollision:
        return {ErrorCode::SelfCollision, "SimRobotController: planned robot links overlap"};
    case JointStateInvalidity::AttachedObjectCollision:
        return {ErrorCode::AttachedObjectCollision, "SimRobotController: attached object overlaps the scene"};
    }
    return {ErrorCode::Fault, "SimRobotController: unknown joint-state invalidity"};
}

void AlignEquivalentJointAngles(
    JointVector& target,
    const JointVector& reference,
    const models::RobotSpecification& specification)
{
    // 회전 관절은 2π를 더하거나 빼도 같은 자세다. 허용 범위 안에서 현재 각도와 가장 가까운 표현을 골라 불필요한 한 바퀴 회전을 막는다.
    for (std::size_t jointIndex = 0; jointIndex < target.size(); ++jointIndex)
    {
        const auto& joint = specification.joints[jointIndex];
        const double minimumTurns = std::ceil((joint.minPositionRadians - target[jointIndex]) / kFullTurnRadians);
        const double maximumTurns = std::floor((joint.maxPositionRadians - target[jointIndex]) / kFullTurnRadians);
        if (minimumTurns > maximumTurns)
            continue;

        const double nearestTurns = std::round((reference[jointIndex] - target[jointIndex]) / kFullTurnRadians);
        const double legalTurns = std::clamp(nearestTurns, minimumTurns, maximumTurns);
        target[jointIndex] += legalTurns * kFullTurnRadians;
    }
}

std::optional<kinematics::IkResult> SolveCollisionFreeIk(
    kinematics::DampedLeastSquaresIk& inverse,
    const models::RobotSpecification& specification,
    const StateValidityChecker& stateValidityChecker,
    const CartesianPose& target,
    const JointVector& start,
    const kinematics::IkOptions& options,
    const PlanningPolicy& policy,
    JointStateInvalidity& invalidity,
    kinematics::IkResult& ikFailure)
{
    invalidity = JointStateInvalidity::None;
    const auto startInvalidity = ValidateJointState(specification, start, {});
    if (startInvalidity != JointStateInvalidity::None)
    {
        invalidity = startInvalidity;
        return std::nullopt;
    }
    // 먼저 직전 자세를 seed로 풀어 연속성을 유지한다. 다른 seed가 필요하면 시작 자세에서 관절 범위로 정규화한 변화량이 가장 작은 안전한 해를 고른다.
    auto preferred = inverse.SolveSingleSeed(target, start, options);
    bool preferredPathValid = false;
    if (preferred)
    {
        AlignEquivalentJointAngles(preferred.jointPositionRadians, start, specification);
        const auto pathInvalidity = ValidateJointPath(
            start, preferred.jointPositionRadians, specification, stateValidityChecker, policy);
        preferredPathValid = pathInvalidity == JointStateInvalidity::None;
        if (!preferredPathValid)
            invalidity = pathInvalidity;
        else if (!IsApproachingJ1Limit(start, preferred.jointPositionRadians, specification))
            return preferred;
    }
    else
    {
        ikFailure = preferred;
        if (preferred.status != kinematics::IkStatus::DidNotConverge &&
            preferred.status != kinematics::IkStatus::JointLimitReached)
            return std::nullopt;
    }

    // 해는 있지만 모든 해의 경로가 막힌 경우와 목표 자체에 IK 해가 없는 경우를 구분한다.
    std::optional<kinematics::IkResult> bestSolution;
    double bestCandidateScore = std::numeric_limits<double>::infinity();
    const auto residual = [&](const kinematics::IkResult& result)
    {
        return result.positionErrorMeters + options.orientationWeightMetersPerRadian *
            result.orientationErrorRadians;
    };
    double bestFailureResidual = ikFailure.message.empty()
        ? std::numeric_limits<double>::infinity()
        : residual(ikFailure);
    const auto considerSolution = [&](kinematics::IkResult solution, bool pathAlreadyValid = false)
    {
        if (!solution)
        {
            const double candidateResidual = residual(solution);
            if (candidateResidual < bestFailureResidual)
            {
                bestFailureResidual = candidateResidual;
                ikFailure = std::move(solution);
            }
            return;
        }

        AlignEquivalentJointAngles(solution.jointPositionRadians, start, specification);
        double score = 0.0;
        for (std::size_t joint = 0; joint < start.size(); ++joint)
        {
            const auto& limits = specification.joints[joint];
            const double range = std::max(1e-12,
                limits.maxPositionRadians - limits.minPositionRadians);
            const double delta = (solution.jointPositionRadians[joint] - start[joint]) / range;
            score += delta * delta;
            const double marginFraction = std::min(
                solution.jointPositionRadians[joint] - limits.minPositionRadians,
                limits.maxPositionRadians - solution.jointPositionRadians[joint]) / range;
            const double marginDeficit = std::max(0.0,
                kJointLimitSearchMarginFraction - marginFraction) / kJointLimitSearchMarginFraction;
            // 한계 가까이에서는 다음 TCP 표본으로 이어갈 관절 방향이 줄어든다. 지금 작은 우회를 허용해 뒤 표본에서 분기를 급히 바꾸는 상황을 줄인다.
            score += kJointLimitPenaltyWeight * marginDeficit * marginDeficit;
        }
        if (score >= bestCandidateScore)
            return;

        if (!pathAlreadyValid)
        {
            const auto pathInvalidity = ValidateJointPath(
                start, solution.jointPositionRadians, specification, stateValidityChecker, policy);
            if (pathInvalidity != JointStateInvalidity::None)
            {
                if (invalidity == JointStateInvalidity::None)
                    invalidity = pathInvalidity;
                return;
            }
        }

        bestCandidateScore = score;
        bestSolution = std::move(solution);
    };

    if (preferredPathValid)
    {
        considerSolution(std::move(preferred), true);
        JointVector alternateJ1Seed = start;
        const auto& j1Limits = specification.joints[0];
        alternateJ1Seed.front() = std::clamp(
            j1Limits.minPositionRadians + j1Limits.maxPositionRadians - start.front(),
            j1Limits.minPositionRadians, j1Limits.maxPositionRadians);
        considerSolution(inverse.SolveSingleSeed(target, alternateJ1Seed, options));
    }
    else
    {
        for (const auto& seed : kinematics::detail::BuildAlternativeIkSeeds(start, specification))
            considerSolution(inverse.SolveSingleSeed(target, seed, options));
    }

    if (bestSolution)
        return bestSolution;

    // 기존 자세와 반사 분기에서 해를 찾지 못했을 때만 현재 자세를 기준으로 재시작 seed를 시험한다.
    JointVector restartSeed(start.size());
    for (std::size_t sample = 1; sample <= kIkRestartSeedCount; ++sample)
    {
        BuildRestartSeed(sample, start, specification, restartSeed);
        considerSolution(inverse.SolveSingleSeed(target, restartSeed, options));
    }

    if (bestSolution)
        return bestSolution;
    return std::nullopt;
}

Result BuildLinearPath(
    const LinearPathMoveCommand& command,
    const models::RobotSpecification& specification,
    const JointVector& startJoints,
    const CartesianPose& startTcp,
    kinematics::DampedLeastSquaresIk& inverse,
    const StateValidityChecker& stateValidityChecker,
    LinearPathPlan& plan,
    const PlanningPolicy& policy)
{
    using namespace kinematics::detail;

    if (!IsPolicyValid(policy))
        return {ErrorCode::InvalidCommand, "SimRobotController: invalid linear path planning policy"};

    std::vector<Pose3> targets;
    targets.reserve(command.targetPoses.size());
    try
    {
        for (const auto& pose : command.targetPoses)
            targets.push_back(FromCartesian(pose));
    }
    catch (const std::invalid_argument&)
    {
        return {ErrorCode::InvalidCommand, "SimRobotController: invalid TCP target"};
    }

    const Pose3 start = FromCartesian(startTcp);
    const auto options = PathIkOptions();
    Pose3 segmentStart = start;
    LinearPathPlan candidatePlan;
    for (const Pose3& end : targets)
    {
        candidatePlan.hasMotion = candidatePlan.hasMotion ||
            Length(Subtract(end.positionMeters, segmentStart.positionMeters)) > options.positionToleranceMeters ||
            Length(RotationError(end.rotation, segmentStart.rotation)) > options.orientationToleranceRadians;
        segmentStart = end;
    }
    if (!candidatePlan.hasMotion)
    {
        plan = std::move(candidatePlan);
        return Result::Success();
    }

    std::vector<Pose3> tcpSamples;
    tcpSamples.reserve(64);
    std::size_t totalIntervals = 0;
    segmentStart = start;
    for (const Pose3& end : targets)
    {
        const double distance = Length(Subtract(end.positionMeters, segmentStart.positionMeters));
        const double rotation = Length(RotationError(end.rotation, segmentStart.rotation));
        const double intervals = std::max({1.0,
            std::ceil(distance / policy.linearPositionSampleSpacingMeters),
            std::ceil(rotation / policy.linearOrientationSampleSpacingRadians)});
        // 표본 수 초과는 먼저 끝점만 풀어 도달 불가와 계획 해상도 한도 초과를 서로 다른 오류로 반환한다.
        if (!std::isfinite(intervals) || intervals > static_cast<double>(policy.maximumPathIntervals) ||
            totalIntervals > policy.maximumPathIntervals ||
            intervals > static_cast<double>(policy.maximumPathIntervals - totalIntervals))
        {
            // 샘플 한도를 넘었어도 끝점 IK는 따로 확인해 도달 불가와 경로 해상도 제한을 구분한다.
            const auto endpoint = inverse.Solve(ToCartesian(end), startJoints);
            if (!endpoint)
                return MapIkFailure(endpoint);
            return {ErrorCode::InvalidCommand, "SimRobotController: linear path exceeds " +
                std::to_string(policy.maximumPathIntervals) + " intervals"};
        }

        const std::size_t count = static_cast<std::size_t>(intervals);
        totalIntervals += count;
        for (std::size_t i = 1; i <= count; ++i)
        {
            const double fraction = static_cast<double>(i) / static_cast<double>(count);
            tcpSamples.push_back(Interpolate(segmentStart, end, fraction));
        }
        segmentStart = end;
    }

    std::vector<std::vector<PathCandidate>> layers;
    layers.reserve(policy.maximumPathIntervals + 1);
    layers.push_back({PathCandidate{startJoints, start, 0.0, 0}});
    std::vector<std::size_t> refinementDepth(tcpSamples.size(), 0);
    std::size_t sample = 0;
    while (sample < tcpSamples.size())
    {
        bool needsRefinement = false;
        JointStateInvalidity invalidity = JointStateInvalidity::None;
        kinematics::IkResult ikFailure;
        auto candidates = BuildNextCandidates(sample, tcpSamples[sample], layers.back(),
            inverse, specification, stateValidityChecker, options, policy,
            needsRefinement, invalidity, ikFailure);
        if (candidates.empty())
        {
            const CartesianPose target = ToCartesian(tcpSamples[sample]);
            if (needsRefinement)
            {
                if (refinementDepth[sample] >= policy.maximumTcpRefinementPasses ||
                    tcpSamples.size() >= policy.maximumPathIntervals)
                    return {ErrorCode::IkDidNotConverge,
                        "SimRobotController: TCP straightness error remains above tolerance at sample " +
                            std::to_string(sample + 1) + "/" + std::to_string(tcpSamples.size()) +
                            " after " + std::to_string(refinementDepth[sample]) + " refinements"};
                const Pose3& previousPose = layers.back().front().tcpPose;
                const Pose3 midpoint = Interpolate(previousPose, tcpSamples[sample], 0.5);
                const std::size_t nextDepth = refinementDepth[sample] + 1;
                tcpSamples.insert(tcpSamples.begin() + static_cast<std::ptrdiff_t>(sample), midpoint);
                refinementDepth[sample] = nextDepth;
                refinementDepth.insert(refinementDepth.begin() + static_cast<std::ptrdiff_t>(sample + 1), nextDepth);
                continue;
            }
            const std::size_t sampleNumber = sample + 1;
            const std::size_t sampleCount = tcpSamples.size();
            if (invalidity == JointStateInvalidity::EnvironmentCollision)
                return AddPathSampleContext(
                    {ErrorCode::EnvironmentContact, "SimRobotController: no collision-free IK solution"},
                    sampleNumber, sampleCount, specification, target);
            if (invalidity != JointStateInvalidity::None)
                return AddPathSampleContext(MapJointStateInvalidity(invalidity), sampleNumber,
                    sampleCount, specification, target);
            return AddPathSampleContext(MapIkFailure(ikFailure), sampleNumber, sampleCount,
                specification, target, &ikFailure);
        }
        layers.push_back(std::move(candidates));
        ++sample;
    }

    std::size_t selected = static_cast<std::size_t>(std::min_element(
        layers.back().begin(), layers.back().end(), [](const PathCandidate& left, const PathCandidate& right)
        {
            return left.score < right.score;
        }) - layers.back().begin());
    std::vector<JointVector> selectedJoints(tcpSamples.size());
    for (std::size_t layer = tcpSamples.size(); layer > 0; --layer)
    {
        const auto& node = layers[layer][selected];
        selectedJoints[layer - 1] = node.joints;
        selected = node.previousCandidate;
    }

    candidatePlan.points.clear();
    candidatePlan.points.reserve(tcpSamples.size() + 1);
    candidatePlan.points.push_back({startJoints, ToCartesian(start), 0.0});
    candidatePlan.plannedLinearVelocity = 0.0;
    candidatePlan.plannedAngularVelocity = 0.0;
    for (std::size_t point = 0; point < tcpSamples.size(); ++point)
    {
        const auto& previousPoint = candidatePlan.points.back();
        const double distance = Length(Subtract(tcpSamples[point].positionMeters,
            FromCartesian(previousPoint.tcpPose).positionMeters));
        const double rotation = Length(RotationError(tcpSamples[point].rotation,
            FromCartesian(previousPoint.tcpPose).rotation));
        double duration = std::max(
            RequiredTimeForVelocity(distance, command.maxLinearVelocityMetersPerSecond),
            RequiredTimeForVelocity(rotation, command.maxAngularVelocityRadiansPerSecond));
        for (std::size_t joint = 0; joint < specification.jointCount; ++joint)
            duration = std::max(duration, RequiredTimeForVelocity(
                selectedJoints[point][joint] - previousPoint.joints[joint],
                specification.joints[joint].maxVelocityRadiansPerSecond));
        if (!std::isfinite(duration))
            return {ErrorCode::InvalidCommand, "SimRobotController: path duration exceeds numeric range"};
        candidatePlan.points.push_back({std::move(selectedJoints[point]), ToCartesian(tcpSamples[point]),
            std::max(duration, 1e-6)});
        candidatePlan.plannedLinearVelocity = std::max(candidatePlan.plannedLinearVelocity,
            distance / duration);
        candidatePlan.plannedAngularVelocity = std::max(candidatePlan.plannedAngularVelocity,
            rotation / duration);
    }
    plan = std::move(candidatePlan);
    return Result::Success();
}

} // namespace grasplink::robotics::planning
