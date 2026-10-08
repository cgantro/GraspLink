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

#if GRASPLINK_ENABLE_TRACY
#include <tracy/Tracy.hpp>
#else
#define ZoneScopedN(name) ((void)0)
#endif

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

struct IkSeedCandidate
{
    std::size_t previousCandidate = 0;
    JointVector joints;
    bool continuation = false;
};

struct EvaluatedEdge
{
    std::size_t parent = 0;
    JointVector joints;
    bool followsTcp = false;
};

}

struct LinearPathPlanningJob::Impl
{
    enum class Phase { PrepareSample, SolveSeed, CheckTcp, ValidatePath, FinishSample };
    enum class ValidationPurpose { Candidate, RefinementProbe };

    LinearPathMoveCommand command;
    const models::RobotSpecification* specification = nullptr;
    JointVector startJoints;
    kinematics::DampedLeastSquaresIk* inverse = nullptr;
    StateValidityChecker stateValidityChecker;
    PlanningPolicy policy{};
    kinematics::IkOptions options{};
    Pose3 start{};
    std::vector<Pose3> targets;
    std::vector<Pose3> tcpSamples;
    std::vector<std::vector<PathCandidate>> layers;
    std::vector<std::size_t> refinementDepth;
    std::size_t sample = 0;
    LinearPathPlanningState state = LinearPathPlanningState::Idle;
    Result result{};
    LinearPathPlan plan{};

    std::vector<IkSeedCandidate> seeds;
    std::vector<PathCandidate> candidates;
    std::vector<EvaluatedEdge> evaluatedEdges;
    std::vector<bool> parentHasContinuation;
    std::vector<bool> parentNeedsSearch;
    std::vector<bool> parentHasValidChild;
    std::size_t seedIndex = 0;
    std::size_t tcpProbe = 0;
    std::size_t refinementEdge = 0;
    double bestFailureResidual = std::numeric_limits<double>::infinity();
    bool needsRefinement = false;
    bool expandedSeeds = false;
    bool planTaken = false;
    bool foundCollisionFreeRefinementEdge = false;
    JointStateInvalidity invalidity = JointStateInvalidity::None;
    JointStateInvalidity rejectedRefinementInvalidity = JointStateInvalidity::None;
    kinematics::IkResult ikFailure;
    kinematics::IkResult workingSolution;
    double workingScore = 0.0;
    std::size_t workingParent = 0;
    bool workingContinuation = false;
    bool workingLimitRisky = false;
    kinematics::DampedLeastSquaresIk::SessionId ikSession = 0;
    Phase phase = Phase::PrepareSample;
    ValidationPurpose validationPurpose = ValidationPurpose::Candidate;
    JointVector validationStart;
    JointVector validationEnd;
    JointVector validationSample;
    std::size_t validationIntervals = 0;
    std::size_t validationStep = 0;

    Result Begin(const LinearPathMoveCommand& moveCommand,
        const models::RobotSpecification& robot, const JointVector& joints,
        const CartesianPose& tcp, kinematics::DampedLeastSquaresIk& ik,
        const StateValidityChecker& checker, const PlanningPolicy& planningPolicy)
    {
        if (ikSession != 0 && inverse != nullptr)
            inverse->CancelSingleSeed(ikSession);
        ikSession = 0;
        stateValidityChecker = {};
        command = moveCommand;
        specification = &robot;
        startJoints = joints;
        inverse = &ik;
        stateValidityChecker = checker;
        policy = planningPolicy;
        plan = {};
        planTaken = false;
        result = Result::Success();
        sample = 0;
        targets.clear();
        tcpSamples.clear();
        layers.clear();
        refinementDepth.clear();
        seeds.clear();
        candidates.clear();
        evaluatedEdges.clear();

        if (!IsPolicyValid(policy))
            return Fail({ErrorCode::InvalidCommand, "SimRobotController: invalid linear path planning policy"});
        targets.reserve(command.targetPoses.size());
        try
        {
            for (const auto& pose : command.targetPoses)
                targets.push_back(kinematics::detail::FromCartesian(pose));
            start = kinematics::detail::FromCartesian(tcp);
        }
        catch (const std::invalid_argument&)
        {
            return Fail({ErrorCode::InvalidCommand, "SimRobotController: invalid TCP target"});
        }

        options = PathIkOptions();
        for (const CartesianPose& endpoint : command.targetPoses)
        {
            const auto endpointSession = inverse->BeginSingleSeed(endpoint, joints, options);
            const auto& endpointCheck = inverse->GetSingleSeedResult(endpointSession);
            if (endpointCheck.status == kinematics::IkStatus::Unreachable)
                return Fail(MapIkFailure(endpointCheck));
            inverse->CancelSingleSeed(endpointSession);
        }

        Pose3 segmentStart = start;
        for (const Pose3& end : targets)
        {
            plan.hasMotion = plan.hasMotion ||
                kinematics::detail::Length(kinematics::detail::Subtract(
                    end.positionMeters, segmentStart.positionMeters)) > options.positionToleranceMeters ||
                kinematics::detail::Length(kinematics::detail::RotationError(
                    end.rotation, segmentStart.rotation)) > options.orientationToleranceRadians;
            segmentStart = end;
        }
        if (!plan.hasMotion)
        {
            state = LinearPathPlanningState::Completed;
            return result;
        }

        tcpSamples.reserve(64);
        std::size_t totalIntervals = 0;
        segmentStart = start;
        for (const Pose3& end : targets)
        {
            const double distance = kinematics::detail::Length(
                kinematics::detail::Subtract(end.positionMeters, segmentStart.positionMeters));
            const double rotation = kinematics::detail::Length(
                kinematics::detail::RotationError(end.rotation, segmentStart.rotation));
            const double intervals = std::max({1.0,
                std::ceil(distance / policy.linearPositionSampleSpacingMeters),
                std::ceil(rotation / policy.linearOrientationSampleSpacingRadians)});
            if (!std::isfinite(intervals) || intervals > static_cast<double>(policy.maximumPathIntervals) ||
                totalIntervals > policy.maximumPathIntervals ||
                intervals > static_cast<double>(policy.maximumPathIntervals - totalIntervals))
            {
                return Fail({ErrorCode::InvalidCommand, "SimRobotController: linear path exceeds " +
                    std::to_string(policy.maximumPathIntervals) + " intervals"});
            }

            const std::size_t count = static_cast<std::size_t>(intervals);
            totalIntervals += count;
            for (std::size_t i = 1; i <= count; ++i)
                tcpSamples.push_back(kinematics::detail::Interpolate(segmentStart, end,
                    static_cast<double>(i) / static_cast<double>(count)));
            segmentStart = end;
        }
        layers.reserve(policy.maximumPathIntervals + 1);
        layers.push_back({PathCandidate{joints, start, 0.0, 0}});
        refinementDepth.resize(tcpSamples.size(), 0);
        state = LinearPathPlanningState::Running;
        phase = Phase::PrepareSample;
        return Result::Success();
    }

    Result Fail(Result failure)
    {
        result = std::move(failure);
        state = LinearPathPlanningState::Failed;
        return result;
    }

    void ReleasePlanningStorage()
    {
        stateValidityChecker = {};
        std::vector<Pose3>().swap(targets);
        std::vector<Pose3>().swap(tcpSamples);
        std::vector<std::vector<PathCandidate>>().swap(layers);
        std::vector<std::size_t>().swap(refinementDepth);
        std::vector<IkSeedCandidate>().swap(seeds);
        std::vector<PathCandidate>().swap(candidates);
        std::vector<EvaluatedEdge>().swap(evaluatedEdges);
        std::vector<bool>().swap(parentHasContinuation);
        std::vector<bool>().swap(parentNeedsSearch);
        std::vector<bool>().swap(parentHasValidChild);
        std::vector<double>().swap(validationSample);
        std::vector<double>().swap(validationStart);
        std::vector<double>().swap(validationEnd);
    }

    void Finish()
    {
        using namespace kinematics::detail;
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

        plan.points.clear();
        plan.points.reserve(tcpSamples.size() + 1);
        plan.points.push_back({startJoints, ToCartesian(start), 0.0});
        plan.plannedLinearVelocity = 0.0;
        plan.plannedAngularVelocity = 0.0;
        for (std::size_t point = 0; point < tcpSamples.size(); ++point)
        {
            const auto& previousPoint = plan.points.back();
            const double distance = Length(Subtract(tcpSamples[point].positionMeters,
                FromCartesian(previousPoint.tcpPose).positionMeters));
            const double rotation = Length(RotationError(tcpSamples[point].rotation,
                FromCartesian(previousPoint.tcpPose).rotation));
            double duration = std::max(
                RequiredTimeForVelocity(distance, command.maxLinearVelocityMetersPerSecond),
                RequiredTimeForVelocity(rotation, command.maxAngularVelocityRadiansPerSecond));
            for (std::size_t joint = 0; joint < specification->jointCount; ++joint)
                duration = std::max(duration, RequiredTimeForVelocity(
                    selectedJoints[point][joint] - previousPoint.joints[joint],
                    specification->joints[joint].maxVelocityRadiansPerSecond));
            if (!std::isfinite(duration))
            {
                Fail({ErrorCode::InvalidCommand, "SimRobotController: path duration exceeds numeric range"});
                return;
            }
            plan.points.push_back({std::move(selectedJoints[point]), ToCartesian(tcpSamples[point]),
                std::max(duration, 1e-6)});
            plan.plannedLinearVelocity = std::max(plan.plannedLinearVelocity, distance / duration);
            plan.plannedAngularVelocity = std::max(plan.plannedAngularVelocity, rotation / duration);
        }
        state = LinearPathPlanningState::Completed;
        result = Result::Success();
    }

    void PrepareSample()
    {
        const auto& previous = layers.back();
        seeds.clear();
        seeds.reserve(std::max(policy.maximumIkSeedAttemptsPerSample, previous.size()));
        for (std::size_t parent = 0; parent < previous.size(); ++parent)
            seeds.push_back({parent, previous[parent].joints, true});
        candidates.clear();
        candidates.reserve(policy.maximumIkCandidatesPerSample);
        evaluatedEdges.clear();
        parentHasContinuation.assign(previous.size(), false);
        parentNeedsSearch.assign(previous.size(), false);
        parentHasValidChild.assign(previous.size(), false);
        seedIndex = 0;
        tcpProbe = 0;
        refinementEdge = 0;
        bestFailureResidual = std::numeric_limits<double>::infinity();
        needsRefinement = false;
        expandedSeeds = false;
        foundCollisionFreeRefinementEdge = false;
        invalidity = JointStateInvalidity::None;
        rejectedRefinementInvalidity = JointStateInvalidity::None;
        ikFailure = {};
        phase = Phase::SolveSeed;
    }

    void StartSeed()
    {
        const auto& seed = seeds[seedIndex];
        const auto target = kinematics::detail::ToCartesian(tcpSamples[sample]);
        ikSession = inverse->BeginSingleSeed(target, seed.joints, options);
        ++plan.ikSolveCount;
        workingParent = seed.previousCandidate;
        workingContinuation = seed.continuation;
    }

    void RecordIkFailure(kinematics::IkResult failure)
    {
        const double residual = failure.positionErrorMeters +
            options.orientationWeightMetersPerRadian * failure.orientationErrorRadians;
        if (residual < bestFailureResidual)
        {
            bestFailureResidual = residual;
            ikFailure = std::move(failure);
        }
        if (workingContinuation)
            parentNeedsSearch[workingParent] = true;
    }

    void PrepareSolution()
    {
        if (!workingSolution)
        {
            RecordIkFailure(std::move(workingSolution));
            ++seedIndex;
            phase = Phase::SolveSeed;
            return;
        }

        const auto& parent = layers.back()[workingParent];
        AlignEquivalentJointAngles(workingSolution.jointPositionRadians, parent.joints, *specification);
        workingScore = parent.score + JointTransitionCost(
            parent.joints, workingSolution.jointPositionRadians, *specification);
        auto duplicate = std::find_if(candidates.begin(), candidates.end(), [&](const PathCandidate& candidate)
        {
            return SameJointCandidate(candidate.joints, workingSolution.jointPositionRadians);
        });
        if (duplicate != candidates.end() && workingScore >= duplicate->score)
        {
            if (duplicate->previousCandidate == workingParent)
            {
                parentHasValidChild[workingParent] = true;
                if (workingContinuation)
                {
                    parentHasContinuation[workingParent] = true;
                    duplicate->stableSamples = std::max(duplicate->stableSamples,
                        parent.stableSamples + 1);
                }
            }
            ++seedIndex;
            phase = Phase::SolveSeed;
            return;
        }
        if (candidates.size() >= policy.maximumIkCandidatesPerSample)
        {
            const auto worst = std::max_element(candidates.begin(), candidates.end(),
                [](const PathCandidate& left, const PathCandidate& right)
                {
                    return left.score < right.score;
                });
            if (workingScore > worst->score)
            {
                ++seedIndex;
                phase = Phase::SolveSeed;
                return;
            }
        }

        for (const auto& edge : evaluatedEdges)
        {
            if (edge.parent == workingParent &&
                std::equal(workingSolution.jointPositionRadians.begin(),
                    workingSolution.jointPositionRadians.end(), edge.joints.begin()))
            {
                if (workingContinuation)
                    parentHasContinuation[workingParent] = true;
                ++seedIndex;
                phase = Phase::SolveSeed;
                return;
            }
        }
        evaluatedEdges.push_back({workingParent, workingSolution.jointPositionRadians, false});
        tcpProbe = 0;
        phase = Phase::CheckTcp;
    }

    void FinishTcpCheck()
    {
        auto& edge = evaluatedEdges.back();
        const auto& parent = layers.back()[workingParent];
        if (tcpProbe < 3)
            return;
        edge.followsTcp = true;
        workingLimitRisky = IsLimitRisky(parent.joints, workingSolution.jointPositionRadians, *specification);
        BeginPathValidation(parent.joints, workingSolution.jointPositionRadians, ValidationPurpose::Candidate);
    }

    void BeginPathValidation(const JointVector& pathStart, const JointVector& pathEnd,
        ValidationPurpose purpose)
    {
        validationPurpose = purpose;
        validationStart = pathStart;
        validationEnd = pathEnd;
        validationStep = 1;
        validationSample.resize(pathStart.size());
        ++plan.jointPathValidityChecks;
        const auto startInvalidity = ValidateJointState(*specification, validationStart, {});
        const auto endInvalidity = ValidateJointState(*specification, validationEnd, {});
        if (startInvalidity != JointStateInvalidity::None || endInvalidity != JointStateInvalidity::None)
        {
            CompletePathValidation(startInvalidity != JointStateInvalidity::None ? startInvalidity : endInvalidity);
            return;
        }
        double maximumJointChange = 0.0;
        for (std::size_t joint = 0; joint < validationStart.size(); ++joint)
            maximumJointChange = std::max(maximumJointChange,
                std::abs(validationEnd[joint] - validationStart[joint]));
        validationIntervals = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(
            maximumJointChange / policy.jointCollisionSampleSpacingRadians)));
        phase = Phase::ValidatePath;
    }

    void CompletePathValidation(JointStateInvalidity pathInvalidity)
    {
        if (validationPurpose == ValidationPurpose::RefinementProbe)
        {
            if (pathInvalidity == JointStateInvalidity::None)
                foundCollisionFreeRefinementEdge = true;
            else if (rejectedRefinementInvalidity == JointStateInvalidity::None)
                rejectedRefinementInvalidity = pathInvalidity;
            ++refinementEdge;
            phase = Phase::FinishSample;
            return;
        }

        if (pathInvalidity != JointStateInvalidity::None)
        {
            if (invalidity == JointStateInvalidity::None)
                invalidity = pathInvalidity;
            parentNeedsSearch[workingParent] = true;
            if (workingContinuation)
                parentHasContinuation[workingParent] = true;
            ++seedIndex;
            phase = Phase::SolveSeed;
            return;
        }

        const auto& parent = layers.back()[workingParent];
        parentHasValidChild[workingParent] = true;
        auto duplicate = std::find_if(candidates.begin(), candidates.end(), [&](const PathCandidate& candidate)
        {
            return SameJointCandidate(candidate.joints, workingSolution.jointPositionRadians);
        });
        if (duplicate == candidates.end())
        {
            candidates.push_back({std::move(workingSolution.jointPositionRadians), tcpSamples[sample],
                workingScore, workingParent, workingLimitRisky ? 0 : parent.stableSamples + 1});
            duplicate = std::prev(candidates.end());
        }
        else if (workingScore < duplicate->score)
        {
            duplicate->joints = std::move(workingSolution.jointPositionRadians);
            duplicate->tcpPose = tcpSamples[sample];
            duplicate->score = workingScore;
            duplicate->previousCandidate = workingParent;
            duplicate->stableSamples = workingLimitRisky ? 0 : parent.stableSamples + 1;
        }
        else if (workingContinuation)
            duplicate->stableSamples = std::max(duplicate->stableSamples, parent.stableSamples + 1);
        if (workingContinuation)
            parentHasContinuation[workingParent] = true;
        if (workingLimitRisky)
        {
            parentNeedsSearch[workingParent] = true;
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
        ++seedIndex;
        phase = Phase::SolveSeed;
    }

    void FinishContinuationSeeds()
    {
        const auto& previous = layers.back();
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
                    previous[parent].joints, *specification);
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
                BuildRestartSeed((sample + 1) * kIkRestartSeedCount + restart,
                    previous[parent].joints, *specification, seed);
                seeds.push_back({parent, std::move(seed), false});
                addedSeed = true;
            }
            if (!addedSeed)
                break;
        }
        seedIndex = previous.size();
        expandedSeeds = true;
        phase = Phase::SolveSeed;
    }

    void FinishCandidateSearch()
    {
        if (candidates.empty() && needsRefinement)
        {
            refinementEdge = 0;
            phase = Phase::FinishSample;
            StartNextRefinementValidation();
            return;
        }
        FinishLayer();
    }

    void StartNextRefinementValidation()
    {
        while (refinementEdge < evaluatedEdges.size() && evaluatedEdges[refinementEdge].followsTcp)
            ++refinementEdge;
        if (refinementEdge < evaluatedEdges.size())
        {
            const auto& edge = evaluatedEdges[refinementEdge];
            BeginPathValidation(layers.back()[edge.parent].joints, edge.joints,
                ValidationPurpose::RefinementProbe);
        }
        else
        {
            needsRefinement = foundCollisionFreeRefinementEdge;
            invalidity = foundCollisionFreeRefinementEdge ? JointStateInvalidity::None : rejectedRefinementInvalidity;
            FinishLayer();
        }
    }

    void FinishLayer()
    {
        using namespace kinematics::detail;
        if (candidates.empty())
        {
            const CartesianPose target = ToCartesian(tcpSamples[sample]);
            if (needsRefinement)
            {
                if (refinementDepth[sample] >= policy.maximumTcpRefinementPasses ||
                    tcpSamples.size() >= policy.maximumPathIntervals)
                {
                    Fail({ErrorCode::IkDidNotConverge,
                        "SimRobotController: TCP straightness error remains above tolerance at sample " +
                            std::to_string(sample + 1) + "/" + std::to_string(tcpSamples.size()) +
                            " after " + std::to_string(refinementDepth[sample]) + " refinements"});
                    return;
                }
                {
                    ZoneScopedN("PlannerRefinement");
                    const Pose3 midpoint = Interpolate(layers.back().front().tcpPose, tcpSamples[sample], 0.5);
                    ++plan.tcpRefinementCount;
                    const std::size_t nextDepth = refinementDepth[sample] + 1;
                    tcpSamples.insert(tcpSamples.begin() + static_cast<std::ptrdiff_t>(sample), midpoint);
                    refinementDepth[sample] = nextDepth;
                    refinementDepth.insert(refinementDepth.begin() + static_cast<std::ptrdiff_t>(sample + 1), nextDepth);
                }
                phase = Phase::PrepareSample;
                return;
            }
            const std::size_t sampleNumber = sample + 1;
            const std::size_t sampleCount = tcpSamples.size();
            if (invalidity == JointStateInvalidity::EnvironmentCollision)
                Fail(AddPathSampleContext(
                    {ErrorCode::EnvironmentContact, "SimRobotController: no collision-free IK solution"},
                    sampleNumber, sampleCount, *specification, target));
            else if (invalidity != JointStateInvalidity::None)
                Fail(AddPathSampleContext(MapJointStateInvalidity(invalidity), sampleNumber,
                    sampleCount, *specification, target));
            else
                Fail(AddPathSampleContext(MapIkFailure(ikFailure), sampleNumber, sampleCount,
                    *specification, target, &ikFailure));
            return;
        }
        std::sort(candidates.begin(), candidates.end(), [](const PathCandidate& left, const PathCandidate& right)
        {
            return left.score < right.score;
        });
        layers.push_back(std::move(candidates));
        ++sample;
        phase = Phase::PrepareSample;
        if (sample == tcpSamples.size())
            Finish();
    }

    void Advance(std::size_t workBudget)
    {
        std::size_t work = 0;
        while (state == LinearPathPlanningState::Running && work < workBudget)
        {
            if (phase == Phase::PrepareSample)
            {
                if (sample >= tcpSamples.size())
                {
                    Finish();
                    continue;
                }
                PrepareSample();
                continue;
            }
            if (phase == Phase::SolveSeed)
            {
                if (seedIndex >= seeds.size())
                {
                    if (!expandedSeeds)
                    {
                        FinishContinuationSeeds();
                        continue;
                    }
                    FinishCandidateSearch();
                    continue;
                }
                if (ikSession == 0)
                    StartSeed();
                {
                    ZoneScopedN("PlannerIK");
                    const auto before = inverse->GetSingleSeedResult(ikSession).iterations;
                    const std::size_t remainingBudget = workBudget - work;
                    const auto ikState = inverse->StepSingleSeed(ikSession, remainingBudget);
                    const auto after = inverse->GetSingleSeedResult(ikSession).iterations;
                    const std::size_t iterationsUsed = after - before;
                    plan.ikIterationCount += iterationsUsed;
                    std::size_t consumedWork = iterationsUsed;
                    if (ikState != kinematics::IkSessionState::Running && after < options.maxIterations)
                        ++consumedWork;
                    if (consumedWork == 0)
                        consumedWork = 1;
                    work += consumedWork;
                    if (ikState == kinematics::IkSessionState::Running)
                        continue;
                    workingSolution = inverse->GetSingleSeedResult(ikSession);
                }
                ikSession = 0;
                PrepareSolution();
                continue;
            }
            if (phase == Phase::CheckTcp)
            {
                const auto& parent = layers.back()[workingParent];
                JointVector joints(parent.joints.size());
                const double fraction = 0.25 * static_cast<double>(tcpProbe + 1);
                for (std::size_t joint = 0; joint < joints.size(); ++joint)
                    joints[joint] = parent.joints[joint] +
                        (workingSolution.jointPositionRadians[joint] - parent.joints[joint]) * fraction;
                ZoneScopedN("PlannerTcpStraightness");
                const Pose3 actual = kinematics::detail::FromCartesian(inverse->EvaluateTcp(joints));
                const Pose3 expected = kinematics::detail::Interpolate(parent.tcpPose, tcpSamples[sample], fraction);
                ++plan.tcpStraightnessChecks;
                ++tcpProbe;
                ++work;
                if (kinematics::detail::Length(kinematics::detail::Subtract(
                        actual.positionMeters, expected.positionMeters)) > policy.maximumLinearTcpErrorMeters ||
                    kinematics::detail::Length(kinematics::detail::RotationError(
                        actual.rotation, expected.rotation)) > policy.maximumAngularTcpErrorRadians)
                {
                    needsRefinement = true;
                    parentNeedsSearch[workingParent] = true;
                    if (workingContinuation)
                        parentHasContinuation[workingParent] = true;
                    ++seedIndex;
                    phase = Phase::SolveSeed;
                    continue;
                }
                if (tcpProbe == 3)
                    FinishTcpCheck();
                continue;
            }
            if (phase == Phase::ValidatePath)
            {
                if (validationStep > validationIntervals)
                {
                    CompletePathValidation(JointStateInvalidity::None);
                    if (validationPurpose == ValidationPurpose::RefinementProbe)
                        StartNextRefinementValidation();
                    continue;
                }
                const double fraction = static_cast<double>(validationStep) /
                    static_cast<double>(validationIntervals);
                for (std::size_t joint = 0; joint < validationSample.size(); ++joint)
                    validationSample[joint] = validationStart[joint] +
                        (validationEnd[joint] - validationStart[joint]) * fraction;
                ZoneScopedN("PlannerJointPathValidity");
                const auto pathInvalidity = stateValidityChecker ?
                    stateValidityChecker(validationSample) : JointStateInvalidity::None;
                ++plan.validityStateChecks;
                ++validationStep;
                ++work;
                if (pathInvalidity != JointStateInvalidity::None)
                {
                    CompletePathValidation(pathInvalidity);
                    if (validationPurpose == ValidationPurpose::RefinementProbe)
                        StartNextRefinementValidation();
                }
                continue;
            }
            if (phase == Phase::FinishSample)
            {
                if (refinementEdge < evaluatedEdges.size())
                    StartNextRefinementValidation();
                else
                    FinishLayer();
            }
        }
    }
};

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

LinearPathPlanningJob::LinearPathPlanningJob() : impl_(std::make_unique<Impl>()) {}
LinearPathPlanningJob::~LinearPathPlanningJob() = default;
LinearPathPlanningJob::LinearPathPlanningJob(LinearPathPlanningJob&&) noexcept = default;
LinearPathPlanningJob& LinearPathPlanningJob::operator=(LinearPathPlanningJob&&) noexcept = default;

Result LinearPathPlanningJob::Begin(const LinearPathMoveCommand& command,
    const models::RobotSpecification& specification, const JointVector& startJoints,
    const CartesianPose& startTcp, kinematics::DampedLeastSquaresIk& inverse,
    const StateValidityChecker& stateValidityChecker, const PlanningPolicy& policy)
{
    return impl_->Begin(command, specification, startJoints, startTcp, inverse, stateValidityChecker, policy);
}

LinearPathPlanningState LinearPathPlanningJob::Advance(std::size_t workBudget)
{
    impl_->Advance(workBudget);
    return impl_->state;
}

void LinearPathPlanningJob::Cancel() noexcept
{
    if (impl_->state == LinearPathPlanningState::Running && impl_->ikSession != 0)
    {
        try { impl_->inverse->CancelSingleSeed(impl_->ikSession); }
        catch (...) {}
        impl_->ikSession = 0;
    }
    if (impl_->state == LinearPathPlanningState::Running)
        impl_->state = LinearPathPlanningState::Cancelled;
    impl_->ReleasePlanningStorage();
}

LinearPathPlanningState LinearPathPlanningJob::GetState() const noexcept { return impl_->state; }
const Result& LinearPathPlanningJob::GetResult() const noexcept { return impl_->result; }
const LinearPathPlan& LinearPathPlanningJob::GetPlan() const noexcept { return impl_->plan; }

std::optional<LinearPathPlan> LinearPathPlanningJob::TakePlan()
{
    if (impl_->state != LinearPathPlanningState::Completed || impl_->planTaken)
        return std::nullopt;
    impl_->planTaken = true;
    LinearPathPlan plan = std::move(impl_->plan);
    impl_->plan = {};
    impl_->ReleasePlanningStorage();
    return plan;
}

Result BuildLinearPath(const LinearPathMoveCommand& command,
    const models::RobotSpecification& specification, const JointVector& startJoints,
    const CartesianPose& startTcp, kinematics::DampedLeastSquaresIk& inverse,
    const StateValidityChecker& stateValidityChecker, LinearPathPlan& plan,
    const PlanningPolicy& policy)
{
    LinearPathPlanningJob job;
    const Result begin = job.Begin(command, specification, startJoints, startTcp,
        inverse, stateValidityChecker, policy);
    if (!begin)
        return begin;
    while (job.GetState() == LinearPathPlanningState::Running)
        job.Advance(std::numeric_limits<std::size_t>::max());
    if (job.GetState() == LinearPathPlanningState::Completed)
    {
        auto completedPlan = job.TakePlan();
        if (completedPlan)
            plan = std::move(*completedPlan);
    }
    return job.GetResult();
}

} // namespace grasplink::robotics::planning
