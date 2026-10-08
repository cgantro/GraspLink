#include "robotics/backends/simulation/detail/LinearPathPlanner.h"

#include "robotics/kinematics/detail/PoseMath.h"
#include "robotics/kinematics/detail/AlternativeIkSeeds.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

namespace grasplink::robotics::backends::simulation::detail
{
namespace
{
constexpr double kFullTurnRadians = 2.0 * 3.14159265358979323846;

JointStateInvalidity ValidateJointPath(
    const JointVector& start,
    const JointVector& end,
    const models::RobotSpecification& specification,
    const std::function<JointStateInvalidity(const JointVector&)>& stateValidityChecker,
    const SimulationMotionPolicy& policy)
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
}

JointStateInvalidity ValidateJointState(
    const models::RobotSpecification& specification,
    const JointVector& joints,
    const std::function<JointStateInvalidity(const JointVector&)>& stateValidityChecker)
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
    const std::function<JointStateInvalidity(const JointVector&)>& stateValidityChecker,
    const CartesianPose& target,
    const JointVector& start,
    const kinematics::IkOptions& options,
    const SimulationMotionPolicy& policy,
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
    // 먼저 직전 자세를 seed로 풀어 연속성을 유지한다. 이 해가 없거나 경로가 충돌하면 대체 seed도 검사해 가장 적게 움직이는 안전한 해를 고른다.
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
        ikFailure = preferred;

    // 해는 있지만 모든 해의 경로가 막힌 경우와 목표 자체에 IK 해가 없는 경우를 구분한다.
    std::optional<kinematics::IkResult> bestSolution;
    double bestNormalizedDistance = std::numeric_limits<double>::infinity();
    for (const auto& seed : kinematics::detail::BuildAlternativeIkSeeds(start, specification))
    {
        auto solution = inverse.SolveSingleSeed(target, seed, options);
        if (!solution)
        {
            ikFailure = std::move(solution);
            continue;
        }

        AlignEquivalentJointAngles(solution.jointPositionRadians, start, specification);
        const auto pathInvalidity = ValidateJointPath(
            start, solution.jointPositionRadians, specification, stateValidityChecker, policy);
        if (pathInvalidity != JointStateInvalidity::None)
        {
            if (invalidity == JointStateInvalidity::None)
                invalidity = pathInvalidity;
            continue;
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
    const std::function<JointStateInvalidity(const JointVector&)>& stateValidityChecker,
    LinearPathPlan& plan,
    const SimulationMotionPolicy& policy)
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
                    return {ErrorCode::EnvironmentContact, "SimRobotController: no collision-free IK solution for the TCP path"};
                if (invalidity != JointStateInvalidity::None)
                    return MapJointStateInvalidity(invalidity);
                return MapIkFailure(ikFailure);
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

} // namespace grasplink::robotics::backends::simulation::detail
