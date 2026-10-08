#include "robotics/planning/LinearPathPlanner.h"

#include "robotics/kinematics/detail/PoseMath.h"
#include "robotics/kinematics/RobotInverseKinematics.h"
#include "robotics/models/hanwha/Hcr12a.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>
#include <vector>

using namespace grasplink::robotics;
using namespace grasplink::robotics::planning;
using grasplink::robotics::kinematics::DampedLeastSquaresIk;

namespace
{
double PositionError(const CartesianPose& left, const CartesianPose& right)
{
    double squared = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis)
    {
        const double difference = left.positionMeters[axis] - right.positionMeters[axis];
        squared += difference * difference;
    }
    return std::sqrt(squared);
}

double OrientationError(const CartesianPose& left, const CartesianPose& right)
{
    double dot = 0.0;
    for (std::size_t axis = 0; axis < 4; ++axis)
        dot += left.orientationXyzw[axis] * right.orientationXyzw[axis];
    return 2.0 * std::acos(std::clamp(std::abs(dot), 0.0, 1.0));
}

std::vector<CartesianPose> BuildSamples(const CartesianPose& start, const CartesianPose& end,
    const PlanningPolicy& policy)
{
    const double distance = PositionError(start, end);
    const double rotation = OrientationError(start, end);
    const std::size_t count = std::max<std::size_t>(1, static_cast<std::size_t>(std::max(
        std::ceil(distance / policy.linearPositionSampleSpacingMeters),
        std::ceil(rotation / policy.linearOrientationSampleSpacingRadians))));
    std::vector<CartesianPose> samples;
    samples.reserve(count);
    const auto startPose = grasplink::robotics::kinematics::detail::FromCartesian(start);
    const auto endPose = grasplink::robotics::kinematics::detail::FromCartesian(end);
    for (std::size_t i = 1; i <= count; ++i)
    {
        const double fraction = static_cast<double>(i) / static_cast<double>(count);
        samples.push_back(grasplink::robotics::kinematics::detail::ToCartesian(
            grasplink::robotics::kinematics::detail::Interpolate(startPose, endPose, fraction)));
    }
    return samples;
}

bool BuildGreedyPreferredPath(const std::vector<CartesianPose>& samples,
    const StateValidityChecker& checker, std::vector<JointVector>& joints)
{
    const auto& specification = models::hanwha::kHcr12a;
    DampedLeastSquaresIk inverse(specification);
    JointVector current(specification.jointCount, 0.0);
    const auto options = PathIkOptions();
    for (const auto& sample : samples)
    {
        const auto solution = inverse.SolveSingleSeed(sample, current, options);
        if (!solution)
            return false;
        JointVector target = solution.jointPositionRadians;
        AlignEquivalentJointAngles(target, current, specification);
        if (ValidateJointPath(current, target, specification, checker) != JointStateInvalidity::None)
            return false;
        joints.push_back(target);
        current = std::move(target);
    }
    return true;
}

TEST(LinearPathPlanner, SelectsAContinuousAlternateBranchBeforeGreedyBranchIsBlocked)
{
    const auto& specification = models::hanwha::kHcr12a;
    const JointVector startJoints(specification.jointCount, 0.0);
    DampedLeastSquaresIk inverse(specification);
    const CartesianPose startTcp = inverse.EvaluateTcp(startJoints);
    const std::array<JointVector, 1> targets{{
        {0.22, -0.1, 0.08, 0.1, -0.06, 0.12}}};

    bool foundGreedyFailureWithGlobalRecovery = false;
    for (const auto& targetJoints : targets)
    {
        const CartesianPose targetTcp = inverse.EvaluateTcp(targetJoints);
        LinearPathMoveCommand command;
        command.targetPoses.push_back(targetTcp);
        const auto samples = BuildSamples(startTcp, targetTcp, kDefaultPlanningPolicy);
        std::vector<JointVector> greedyJoints;
        if (!BuildGreedyPreferredPath(samples, {}, greedyJoints) || greedyJoints.size() < 4)
            continue;

        for (const std::size_t blockedIndex : {greedyJoints.size() / 2})
        {
            const JointVector blockedBranchState = greedyJoints[blockedIndex];
            const StateValidityChecker checker = [blockedBranchState](const JointVector& candidate)
            {
                double distanceSquared = 0.0;
                for (std::size_t joint = 0; joint < candidate.size(); ++joint)
                {
                    const double difference = candidate[joint] - blockedBranchState[joint];
                    distanceSquared += difference * difference;
                }
                return distanceSquared < 1e-5 * 1e-5 ?
                    JointStateInvalidity::EnvironmentCollision : JointStateInvalidity::None;
            };
            std::vector<JointVector> blockedGreedyPath;
            if (BuildGreedyPreferredPath(samples, checker, blockedGreedyPath))
                continue;

            LinearPathPlan plan;
            const Result result = BuildLinearPath(command, specification, startJoints, startTcp,
                inverse, checker, plan);
            if (!result)
                continue;
            foundGreedyFailureWithGlobalRecovery = true;
            break;
        }
        if (foundGreedyFailureWithGlobalRecovery)
            break;
    }

    ASSERT_TRUE(foundGreedyFailureWithGlobalRecovery)
        << "layered IK candidates retain a safe branch when the closest local branch becomes invalid later";
}

TEST(LinearPathPlanner, RefinesJointInterpolationUntilTcpFollowsTheRequestedLine)
{
    const auto& specification = models::hanwha::kHcr12a;
    DampedLeastSquaresIk inverse(specification);
    const JointVector startJoints(specification.jointCount, 0.0);
    const JointVector targetJoints{0.55, -0.8, 0.7, 0.45, -0.6, 0.3};
    const CartesianPose startTcp = inverse.EvaluateTcp(startJoints);
    const CartesianPose targetTcp = inverse.EvaluateTcp(targetJoints);
    LinearPathMoveCommand command;
    command.targetPoses.push_back(targetTcp);
    PlanningPolicy policy = kDefaultPlanningPolicy;
    policy.maximumLinearTcpErrorMeters = 0.00025;
    policy.maximumAngularTcpErrorRadians = 0.00025;
    LinearPathPlan plan;

    const Result result = BuildLinearPath(command, specification, startJoints, startTcp,
        inverse, {}, plan, policy);

    ASSERT_TRUE(static_cast<bool>(result)) << result.message;
    ASSERT_GT(plan.points.size(), 2u) << "curved joint interpolation adds Cartesian refinement samples";
    for (std::size_t edge = 1; edge < plan.points.size(); ++edge)
    {
        for (std::size_t probe = 1; probe < 20; ++probe)
        {
            const double fraction = static_cast<double>(probe) / 20.0;
            JointVector joints(specification.jointCount);
            for (std::size_t joint = 0; joint < joints.size(); ++joint)
                joints[joint] = plan.points[edge - 1].joints[joint] + fraction *
                    (plan.points[edge].joints[joint] - plan.points[edge - 1].joints[joint]);
            const CartesianPose actualTcp = inverse.EvaluateTcp(joints);
            const CartesianPose expectedTcp = grasplink::robotics::kinematics::detail::ToCartesian(
                grasplink::robotics::kinematics::detail::Interpolate(
                    grasplink::robotics::kinematics::detail::FromCartesian(plan.points[edge - 1].tcpPose),
                    grasplink::robotics::kinematics::detail::FromCartesian(plan.points[edge].tcpPose), fraction));
            ASSERT_LE(PositionError(actualTcp, expectedTcp), policy.maximumLinearTcpErrorMeters + 1e-8)
                << "FK remains inside the TCP position tolerance at an independent 5%-spaced probe";
            ASSERT_LE(OrientationError(actualTcp, expectedTcp), policy.maximumAngularTcpErrorRadians + 1e-8)
                << "FK orientation remains inside tolerance at an independent 5%-spaced probe";
        }
    }
}

TEST(LinearPathPlanner, ReportsBoundedFailureWhenTcpRefinementBudgetIsExhausted)
{
    const auto& specification = models::hanwha::kHcr12a;
    DampedLeastSquaresIk inverse(specification);
    const JointVector startJoints(specification.jointCount, 0.0);
    const JointVector targetJoints{0.8, -0.9, 0.7, 0.6, -0.8, 0.5};
    const CartesianPose startTcp = inverse.EvaluateTcp(startJoints);
    LinearPathMoveCommand command;
    command.targetPoses.push_back(inverse.EvaluateTcp(targetJoints));
    PlanningPolicy policy = kDefaultPlanningPolicy;
    policy.maximumLinearTcpErrorMeters = 1e-12;
    policy.maximumAngularTcpErrorRadians = 1e-12;
    policy.maximumTcpRefinementPasses = 0;
    LinearPathPlan plan;

    const Result result = BuildLinearPath(command, specification, startJoints, startTcp,
        inverse, {}, plan, policy);

    ASSERT_EQ(result.code, ErrorCode::IkDidNotConverge)
        << "the planner rejects a path whose joint interpolation exceeds TCP tolerance without exceeding its refinement budget";
    ASSERT_TRUE(plan.points.empty()) << "failed refinement does not publish a partial path";
}

TEST(LinearPathPlanner, RejectsSeedBudgetSmallerThanCandidateBeam)
{
    const auto& specification = models::hanwha::kHcr12a;
    DampedLeastSquaresIk inverse(specification);
    const JointVector startJoints(specification.jointCount, 0.0);
    const CartesianPose startTcp = inverse.EvaluateTcp(startJoints);
    LinearPathMoveCommand command;
    command.targetPoses.push_back(startTcp);
    PlanningPolicy policy = kDefaultPlanningPolicy;
    policy.maximumIkCandidatesPerSample = 4;
    policy.maximumIkSeedAttemptsPerSample = 3;
    LinearPathPlan plan;

    const Result result = BuildLinearPath(command, specification, startJoints, startTcp,
        inverse, {}, plan, policy);

    ASSERT_EQ(result.code, ErrorCode::InvalidCommand)
        << "the declared IK seed budget must cover each mandatory beam continuation";
    ASSERT_TRUE(plan.points.empty()) << "invalid planner policy does not publish a path";
}
} // namespace
