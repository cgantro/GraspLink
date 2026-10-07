#include "robotics/backends/simulation/detail/LinearPathPlanner.h"

#include "robotics/kinematics/detail/PoseMath.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

namespace grasplink::robotics::backends::simulation::detail
{
Result BuildLinearPath(
    const LinearPathMoveCommand& command,
    const models::RobotSpecification& specification,
    const JointVector& startJoints,
    const CartesianPose& startTcp,
    const CollisionAwareIkSolver& solveCollisionFreeIk,
    const EndpointReachabilitySolver& solveEndpointReachability,
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
            const auto endpoint = solveEndpointReachability(ToCartesian(end), candidatePlan.points.back().joints);
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
            bool collisionBlocked = false;
            kinematics::IkResult ikFailure;
            auto solution = solveCollisionFreeIk(ToCartesian(targetPose), candidatePlan.points.back().joints,
                options, collisionBlocked, ikFailure);
            if (!solution)
                return collisionBlocked ?
                    Result{ErrorCode::EnvironmentContact, "SimRobotController: no collision-free IK solution for the TCP path"} :
                    MapIkFailure(ikFailure);

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
