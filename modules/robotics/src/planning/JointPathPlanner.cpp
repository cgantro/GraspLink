#include "robotics/planning/JointPathPlanner.h"

#include "robotics/planning/LinearPathPlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <string>
#include <thread>
#include <utility>

namespace grasplink::robotics::planning
{
namespace
{
constexpr double kMinimumRange = 1e-12;

bool IsValidOptions(const JointPathPlannerOptions& options)
{
    const auto& policy = options.validationPolicy;
    return options.maximumIterations > 0 && options.maximumNodesPerTree >= 2 &&
        options.maximumShortcutAttempts > 0 && std::isfinite(options.extensionStepFraction) &&
        options.extensionStepFraction > 0.0 && options.extensionStepFraction <= 1.0 &&
        std::isfinite(options.goalBias) && options.goalBias >= 0.0 && options.goalBias <= 1.0 &&
        std::isfinite(policy.jointCollisionSampleSpacingRadians) &&
        policy.jointCollisionSampleSpacingRadians > 0.0;
}

double NormalizedDistance(const JointVector& left, const JointVector& right,
    const models::RobotSpecification& specification)
{
    double squared = 0.0;
    for (std::size_t joint = 0; joint < left.size(); ++joint)
    {
        const auto& limit = specification.joints[joint];
        const double range = std::max(kMinimumRange,
            limit.maxPositionRadians - limit.minPositionRadians);
        const double difference = (right[joint] - left[joint]) / range;
        squared += difference * difference;
    }
    return std::sqrt(squared);
}

JointVector Interpolate(const JointVector& start, const JointVector& end, double fraction)
{
    JointVector result(start.size());
    for (std::size_t joint = 0; joint < start.size(); ++joint)
        result[joint] = start[joint] + (end[joint] - start[joint]) * fraction;
    return result;
}

bool SameState(const JointVector& left, const JointVector& right)
{
    for (std::size_t joint = 0; joint < left.size(); ++joint)
    {
        if (std::abs(left[joint] - right[joint]) > 1e-10)
            return false;
    }
    return true;
}
}

struct JointPathPlanningJob::Impl
{
    struct Node
    {
        JointVector joints;
        std::size_t parent = std::numeric_limits<std::size_t>::max();
    };

    enum class Phase : std::uint8_t { Endpoints, DirectPath, Expand, Shortcut, FinalValidation };
    enum class EdgePurpose : std::uint8_t { Direct, ExpandActive, ExpandConnect, Shortcut, Final };

    struct PendingEdge
    {
        JointVector from;
        JointVector to;
        std::size_t intervals = 1;
        std::size_t nextSample = 1;
        EdgePurpose purpose = EdgePurpose::Direct;
        std::size_t treeNode = 0;
        bool startTree = true;
    };

    const models::RobotSpecification* specification = nullptr;
    JointVector start;
    JointVector goal;
    StateValidityChecker checker;
    JointPathPlannerOptions options{};
    std::mt19937 random;
    std::thread::id ownerThread{};
    std::vector<Node> firstTree;
    std::vector<Node> secondTree;
    std::vector<JointVector> candidatePath;
    JointPathPlan plan;
    Result result{};
    JointPathPlanningState state = JointPathPlanningState::Idle;
    Phase phase = Phase::Endpoints;
    std::optional<PendingEdge> pendingEdge;
    std::size_t endpointIndex = 0;
    std::size_t iteration = 0;
    std::size_t shortcutAttempt = 0;
    std::size_t finalEdge = 0;
    std::size_t validityChecks = 0;
    bool firstTreeAtStart = true;
    bool connecting = false;
    JointVector connectionTarget;
    std::size_t connectionActiveNode = 0;
    bool connectionActiveIsStartTree = true;

    JointStateInvalidity CheckCollision(const JointVector& joints)
    {
        if (!checker)
            return JointStateInvalidity::None;
        ++validityChecks;
        return checker(joints);
    }

    JointStateInvalidity CheckState(const JointVector& joints)
    {
        return ValidateJointState(*specification, joints,
            [this](const JointVector& sample) { return CheckCollision(sample); });
    }

    void StartEdge(const JointVector& from, const JointVector& to, EdgePurpose purpose,
        std::size_t treeNode = 0, bool startTree = true)
    {
        double maximumJointChange = 0.0;
        for (std::size_t joint = 0; joint < from.size(); ++joint)
            maximumJointChange = std::max(maximumJointChange, std::abs(to[joint] - from[joint]));
        const auto intervals = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(
            maximumJointChange / options.validationPolicy.jointCollisionSampleSpacingRadians)));
        pendingEdge = PendingEdge{from, to, intervals, 1, purpose, treeNode, startTree};
    }

    std::size_t Nearest(const std::vector<Node>& tree, const JointVector& target) const
    {
        std::size_t nearest = 0;
        double bestDistance = NormalizedDistance(tree.front().joints, target, *specification);
        for (std::size_t node = 1; node < tree.size(); ++node)
        {
            const double distance = NormalizedDistance(tree[node].joints, target, *specification);
            if (distance < bestDistance)
            {
                nearest = node;
                bestDistance = distance;
            }
        }
        return nearest;
    }

    JointVector RandomTarget()
    {
        std::uniform_real_distribution<double> unit(0.0, 1.0);
        if (unit(random) < options.goalBias)
            return firstTreeAtStart ? goal : start;

        JointVector sample(specification->jointCount);
        for (std::size_t joint = 0; joint < sample.size(); ++joint)
        {
            const auto& limit = specification->joints[joint];
            sample[joint] = std::uniform_real_distribution<double>(
                limit.minPositionRadians, limit.maxPositionRadians)(random);
        }
        return sample;
    }

    std::vector<JointVector> Trace(const std::vector<Node>& tree, std::size_t node) const
    {
        std::vector<JointVector> path;
        for (;;)
        {
            path.push_back(tree[node].joints);
            if (tree[node].parent == std::numeric_limits<std::size_t>::max())
                break;
            node = tree[node].parent;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    void BuildCandidatePath(std::size_t firstNode, std::size_t secondNode)
    {
        candidatePath = Trace(firstTree, firstNode);
        auto goalToConnection = Trace(secondTree, secondNode);
        std::reverse(goalToConnection.begin(), goalToConnection.end());
        candidatePath.insert(candidatePath.end(), std::next(goalToConnection.begin()), goalToConnection.end());
        phase = Phase::Shortcut;
    }

    void Fail(ErrorCode code, std::string message)
    {
        result = {code, std::move(message)};
        state = JointPathPlanningState::Failed;
        pendingEdge.reset();
    }

    void Finish()
    {
        plan.points = std::move(candidatePath);
        plan.iterations = iteration;
        plan.validityChecks = validityChecks;
        result = Result::Success();
        state = JointPathPlanningState::Completed;
    }

    void AddExtendedNode(const PendingEdge& edge)
    {
        auto& tree = edge.startTree ? firstTree : secondTree;
        tree.push_back({edge.to, edge.treeNode});
        const std::size_t addedNode = tree.size() - 1;
        if (edge.purpose == EdgePurpose::ExpandActive)
        {
            connecting = true;
            connectionTarget = edge.to;
            connectionActiveNode = addedNode;
            connectionActiveIsStartTree = edge.startTree;
            return;
        }

        const bool reached = NormalizedDistance(edge.to, connectionTarget, *specification) <= 1e-10;
        if (reached)
        {
            const std::size_t startNode = connectionActiveIsStartTree ? connectionActiveNode : addedNode;
            const std::size_t goalNode = connectionActiveIsStartTree ? addedNode : connectionActiveNode;
            BuildCandidatePath(startNode, goalNode);
            connecting = false;
        }
    }

    void CompleteEdge(const PendingEdge& edge, JointStateInvalidity invalidity)
    {
        if (edge.purpose == EdgePurpose::Direct)
        {
            if (invalidity == JointStateInvalidity::None)
            {
                plan.points = {start, goal};
                plan.validityChecks = validityChecks;
                state = JointPathPlanningState::Completed;
                return;
            }
            firstTree.push_back({start, std::numeric_limits<std::size_t>::max()});
            secondTree.push_back({goal, std::numeric_limits<std::size_t>::max()});
            phase = Phase::Expand;
            return;
        }

        if (edge.purpose == EdgePurpose::ExpandActive || edge.purpose == EdgePurpose::ExpandConnect)
        {
            if (invalidity == JointStateInvalidity::None)
                AddExtendedNode(edge);
            else if (edge.purpose == EdgePurpose::ExpandConnect)
            {
                connecting = false;
                firstTreeAtStart = !firstTreeAtStart;
            }
            else
                firstTreeAtStart = !firstTreeAtStart;
            return;
        }

        if (edge.purpose == EdgePurpose::Shortcut)
        {
            if (invalidity == JointStateInvalidity::None)
                candidatePath.erase(candidatePath.begin() + static_cast<std::ptrdiff_t>(shortcutFirst + 1),
                    candidatePath.begin() + static_cast<std::ptrdiff_t>(shortcutSecond));
            ++shortcutAttempt;
            return;
        }

        if (invalidity != JointStateInvalidity::None)
        {
            Fail(ErrorCode::Fault, "JointPathPlanner: final path failed revalidation");
            return;
        }
        ++finalEdge;
    }

    void AdvanceOneUnit()
    {
        if (phase == Phase::Endpoints)
        {
            const JointVector& endpoint = endpointIndex == 0 ? start : goal;
            if (CheckState(endpoint) != JointStateInvalidity::None)
            {
                Fail(ErrorCode::InvalidCommand, "JointPathPlanner: start or goal state is invalid");
                return;
            }
            ++endpointIndex;
            if (endpointIndex == 2)
            {
                if (SameState(start, goal))
                {
                    plan.points = {start};
                    plan.validityChecks = validityChecks;
                    state = JointPathPlanningState::Completed;
                }
                else
                    phase = Phase::DirectPath;
            }
            return;
        }

        if (pendingEdge)
        {
            auto& edge = *pendingEdge;
            JointVector sample = Interpolate(edge.from, edge.to,
                static_cast<double>(edge.nextSample) / static_cast<double>(edge.intervals));
            const auto invalidity = CheckState(sample);
            if (invalidity != JointStateInvalidity::None || edge.nextSample == edge.intervals)
            {
                const PendingEdge completed = std::move(edge);
                pendingEdge.reset();
                CompleteEdge(completed, invalidity);
            }
            else
                ++edge.nextSample;
            return;
        }

        switch (phase)
        {
        case Phase::Endpoints:
            break;
        case Phase::DirectPath:
            StartEdge(start, goal, EdgePurpose::Direct);
            break;
        case Phase::Expand:
            ExpandOne();
            break;
        case Phase::Shortcut:
            ShortcutOne();
            break;
        case Phase::FinalValidation:
            ValidateFinalEdge();
            break;
        }
    }

    void ExpandOne()
    {
        if (connecting)
        {
            auto& other = connectionActiveIsStartTree ? secondTree : firstTree;
            if (other.size() >= options.maximumNodesPerTree)
            {
                connecting = false;
                firstTreeAtStart = !firstTreeAtStart;
                return;
            }
            std::size_t nearest = Nearest(other, connectionTarget);
            const double distance = NormalizedDistance(other[nearest].joints, connectionTarget, *specification);
            if (distance <= 1e-10)
            {
                const std::size_t startNode = connectionActiveIsStartTree ? connectionActiveNode : nearest;
                const std::size_t goalNode = connectionActiveIsStartTree ? nearest : connectionActiveNode;
                BuildCandidatePath(startNode, goalNode);
                connecting = false;
                return;
            }
            const double fraction = std::min(1.0, options.extensionStepFraction / distance);
            JointVector next = Interpolate(other[nearest].joints, connectionTarget, fraction);
            StartEdge(other[nearest].joints, next, EdgePurpose::ExpandConnect, nearest,
                !connectionActiveIsStartTree);
            return;
        }

        if (iteration >= options.maximumIterations ||
            (firstTree.size() >= options.maximumNodesPerTree &&
                secondTree.size() >= options.maximumNodesPerTree))
        {
            Fail(ErrorCode::Unreachable, "JointPathPlanner: RRT-Connect budget exhausted");
            return;
        }
        ++iteration;
        auto& active = firstTreeAtStart ? firstTree : secondTree;
        const JointVector target = RandomTarget();
        const std::size_t nearest = Nearest(active, target);
        const double distance = NormalizedDistance(active[nearest].joints, target, *specification);
        if (distance <= 1e-10)
        {
            firstTreeAtStart = !firstTreeAtStart;
            return;
        }
        if (active.size() >= options.maximumNodesPerTree)
        {
            firstTreeAtStart = !firstTreeAtStart;
            return;
        }
        const double fraction = std::min(1.0, options.extensionStepFraction / distance);
        JointVector next = Interpolate(active[nearest].joints, target, fraction);
        StartEdge(active[nearest].joints, next, EdgePurpose::ExpandActive, nearest, firstTreeAtStart);
    }

    std::size_t shortcutFirst = 0;
    std::size_t shortcutSecond = 0;

    void ShortcutOne()
    {
        if (candidatePath.size() < 3 || shortcutAttempt >= options.maximumShortcutAttempts)
        {
            phase = Phase::FinalValidation;
            finalEdge = 0;
            return;
        }
        std::uniform_int_distribution<std::size_t> firstIndex(0, candidatePath.size() - 3);
        shortcutFirst = firstIndex(random);
        std::uniform_int_distribution<std::size_t> secondIndex(shortcutFirst + 2, candidatePath.size() - 1);
        shortcutSecond = secondIndex(random);
        StartEdge(candidatePath[shortcutFirst], candidatePath[shortcutSecond], EdgePurpose::Shortcut);
    }

    void ValidateFinalEdge()
    {
        if (finalEdge + 1 >= candidatePath.size())
        {
            Finish();
            return;
        }
        StartEdge(candidatePath[finalEdge], candidatePath[finalEdge + 1], EdgePurpose::Final);
    }
};

JointPathPlanningJob::JointPathPlanningJob() : impl_(std::make_unique<Impl>()) {}
JointPathPlanningJob::~JointPathPlanningJob() = default;
JointPathPlanningJob::JointPathPlanningJob(JointPathPlanningJob&&) noexcept = default;
JointPathPlanningJob& JointPathPlanningJob::operator=(JointPathPlanningJob&&) noexcept = default;

Result JointPathPlanningJob::Begin(const models::RobotSpecification& specification,
    const JointVector& start, const JointVector& goal,
    const StateValidityChecker& stateValidityChecker, const JointPathPlannerOptions& options)
{
    auto& job = *impl_;
    job.specification = &specification;
    job.start = start;
    job.goal = goal;
    job.checker = stateValidityChecker;
    job.options = options;
    job.ownerThread = std::this_thread::get_id();
    job.random.seed(options.randomSeed);
    job.firstTree.clear();
    job.secondTree.clear();
    job.candidatePath.clear();
    job.plan = {};
    job.endpointIndex = 0;
    job.iteration = 0;
    job.shortcutAttempt = 0;
    job.finalEdge = 0;
    job.validityChecks = 0;
    job.firstTreeAtStart = true;
    job.connecting = false;
    job.pendingEdge.reset();
    job.phase = Impl::Phase::Endpoints;
    job.state = JointPathPlanningState::Running;
    job.result = Result::Success();

    if (specification.joints == nullptr || specification.jointCount == 0 ||
        !IsValidOptions(options) || start.size() != specification.jointCount ||
        goal.size() != specification.jointCount)
    {
        job.Fail(ErrorCode::InvalidCommand, "JointPathPlanner: invalid robot, endpoints, or planner options");
        return job.result;
    }
    if (ValidateJointState(specification, start, {}) != JointStateInvalidity::None ||
        ValidateJointState(specification, goal, {}) != JointStateInvalidity::None)
    {
        job.Fail(ErrorCode::InvalidCommand, "JointPathPlanner: start or goal state is invalid");
        return job.result;
    }
    return Result::Success();
}

JointPathPlanningState JointPathPlanningJob::Advance(std::size_t workBudget)
{
    auto& job = *impl_;
    if (job.state != JointPathPlanningState::Running || workBudget == 0)
        return job.state;
    if (std::this_thread::get_id() != job.ownerThread)
    {
        job.Fail(ErrorCode::InvalidCommand,
            "JointPathPlanner: state validity checks must run on the Begin caller thread");
        return job.state;
    }

    while (workBudget-- > 0 && job.state == JointPathPlanningState::Running)
        job.AdvanceOneUnit();
    return job.state;
}

void JointPathPlanningJob::Cancel() noexcept
{
    if (impl_->state == JointPathPlanningState::Running)
    {
        impl_->state = JointPathPlanningState::Cancelled;
        impl_->result = {ErrorCode::Cancelled, "JointPathPlanner: planning cancelled"};
        impl_->pendingEdge.reset();
    }
}

JointPathPlanningState JointPathPlanningJob::GetState() const noexcept { return impl_->state; }
const Result& JointPathPlanningJob::GetResult() const noexcept { return impl_->result; }
const JointPathPlan& JointPathPlanningJob::GetPlan() const noexcept { return impl_->plan; }

std::optional<JointPathPlan> JointPathPlanningJob::TakePlan()
{
    if (impl_->state != JointPathPlanningState::Completed)
        return std::nullopt;
    auto result = std::move(impl_->plan);
    impl_->plan = {};
    return result;
}

} // namespace grasplink::robotics::planning
