#include "robotics/planning/LinearPathPlanner.h"

#include "robotics/kinematics/detail/PoseMath.h"
#include "robotics/kinematics/detail/AlternativeIkSeeds.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

namespace grasplink::robotics::planning
{
namespace
{
constexpr double kFullTurnRadians = 2.0 * 3.14159265358979323846;
constexpr std::size_t kIkRestartSeedCount = 16;
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
        const auto invalidity = ValidateJointState(specification, sample, stateValidityChecker);
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
    if (preferred)
    {
        AlignEquivalentJointAngles(preferred.jointPositionRadians, start, specification);
        const auto pathInvalidity = ValidateJointPath(
            start, preferred.jointPositionRadians, specification, stateValidityChecker, policy);
        if (pathInvalidity == JointStateInvalidity::None)
            return preferred;
        invalidity = pathInvalidity;
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
    double bestNormalizedDistance = std::numeric_limits<double>::infinity();
    const auto residual = [&](const kinematics::IkResult& result)
    {
        return result.positionErrorMeters + options.orientationWeightMetersPerRadian *
            result.orientationErrorRadians;
    };
    double bestFailureResidual = ikFailure.message.empty()
        ? std::numeric_limits<double>::infinity()
        : residual(ikFailure);
    const auto considerSolution = [&](kinematics::IkResult solution)
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
        const auto pathInvalidity = ValidateJointPath(
            start, solution.jointPositionRadians, specification, stateValidityChecker, policy);
        if (pathInvalidity != JointStateInvalidity::None)
        {
            if (invalidity == JointStateInvalidity::None)
                invalidity = pathInvalidity;
            return;
        }

        double normalizedDistance = 0.0;
        for (std::size_t joint = 0; joint < start.size(); ++joint)
        {
            const double range = std::max(1e-12, specification.joints[joint].maxPositionRadians -
                specification.joints[joint].minPositionRadians);
            const double delta = (solution.jointPositionRadians[joint] - start[joint]) / range;
            normalizedDistance += delta * delta;
        }
        if (normalizedDistance < bestNormalizedDistance)
        {
            bestNormalizedDistance = normalizedDistance;
            bestSolution = std::move(solution);
        }
    };

    for (const auto& seed : kinematics::detail::BuildAlternativeIkSeeds(start, specification))
        considerSolution(inverse.SolveSingleSeed(target, seed, options));

    if (bestSolution)
        return bestSolution;

    // 기존 자세 주변의 seed로 해를 찾지 못했을 때만 관절 범위 전체에 퍼진 재시작 자세를 시험한다.
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

    candidatePlan.points.reserve(64);
    candidatePlan.points.push_back({startJoints, ToCartesian(start), 0.0});
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
            const auto endpoint = inverse.Solve(ToCartesian(end), candidatePlan.points.back().joints);
            if (!endpoint)
                return MapIkFailure(endpoint);
            return {ErrorCode::InvalidCommand, "SimRobotController: linear path exceeds " +
                std::to_string(policy.maximumPathIntervals) + " intervals"};
        }

        const std::size_t count = static_cast<std::size_t>(intervals);
        totalIntervals += count;
        const double positionStep = distance / static_cast<double>(count);
        const double rotationStep = rotation / static_cast<double>(count);
        for (std::size_t i = 1; i <= count; ++i)
        {
            const double fraction = static_cast<double>(i) / static_cast<double>(count);
            const Pose3 targetPose = Interpolate(segmentStart, end, fraction);
            JointStateInvalidity invalidity = JointStateInvalidity::None;
            kinematics::IkResult ikFailure;
            auto solution = SolveCollisionFreeIk(inverse, specification, stateValidityChecker,
                ToCartesian(targetPose), candidatePlan.points.back().joints, options, policy,
                invalidity, ikFailure);
            if (!solution)
            {
                if (invalidity == JointStateInvalidity::EnvironmentCollision)
                    return AddPathSampleContext(
                        {ErrorCode::EnvironmentContact, "SimRobotController: no collision-free IK solution"},
                        i, count, specification, ToCartesian(targetPose));
                if (invalidity != JointStateInvalidity::None)
                    return AddPathSampleContext(MapJointStateInvalidity(invalidity), i, count,
                        specification, ToCartesian(targetPose));
                return AddPathSampleContext(MapIkFailure(ikFailure), i, count, specification,
                    ToCartesian(targetPose), &ikFailure);
            }

            double duration = std::max(
                RequiredTimeForVelocity(positionStep, command.maxLinearVelocityMetersPerSecond),
                RequiredTimeForVelocity(rotationStep, command.maxAngularVelocityRadiansPerSecond));
            for (std::size_t joint = 0; joint < specification.jointCount; ++joint)
                duration = std::max(duration, RequiredTimeForVelocity(
                    solution->jointPositionRadians[joint] - candidatePlan.points.back().joints[joint],
                    specification.joints[joint].maxVelocityRadiansPerSecond));
            if (!std::isfinite(duration))
                return {ErrorCode::InvalidCommand, "SimRobotController: path duration exceeds numeric range"};
            candidatePlan.points.push_back({solution->jointPositionRadians, ToCartesian(targetPose),
                std::max(duration, 1e-6)});
            candidatePlan.plannedLinearVelocity = std::max(candidatePlan.plannedLinearVelocity, positionStep / duration);
            candidatePlan.plannedAngularVelocity = std::max(candidatePlan.plannedAngularVelocity, rotationStep / duration);
        }
        segmentStart = end;
    }

    plan = std::move(candidatePlan);
    return Result::Success();
}

} // namespace grasplink::robotics::planning
