#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/kinematics/detail/PoseMath.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace grasplink::robotics::backends::simulation
{
namespace
{
constexpr double kPositionEpsilon = 1e-8;
constexpr double kFullTurnRadians = 2.0 * 3.14159265358979323846;

void AlignEquivalentJointAngles(
    JointVector& target,
    const JointVector& reference,
    const models::RobotSpecification& specification)
{
    // 회전 관절은 2π를 더하거나 빼도 같은 방향이므로, 제조사 허용 범위 안에서 현재 각도에 가장 가까운 값을 고른다.
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

Result Failure(ErrorCode code, std::string message)
{
    // 잘못된 명령 입력은 예외로 던지지 않고 다른 Controller 구현과 같은 Result 오류 값으로 반환한다.
    return {code, std::move(message)};
}

bool IsScaleValid(double value)
{
    return std::isfinite(value) && value > 0.0 && value <= 1.0;
}

double RequiredTimeForVelocity(double displacement, double maximumVelocity)
{
    return std::abs(displacement) / maximumVelocity;
}

double VelocityRatio(double displacement, double availableSeconds, double maximumVelocity)
{
    return std::abs(displacement) / availableSeconds / maximumVelocity;
}

Result IkFailure(const kinematics::IkResult& result)
{
    using kinematics::IkStatus;
    ErrorCode code = ErrorCode::IkDidNotConverge;
    switch (result.status)
    {
    case IkStatus::Success: return Result::Success();
    case IkStatus::InvalidInput: code = ErrorCode::InvalidCommand; break;
    case IkStatus::MissingToolFrame: code = ErrorCode::Unsupported; break;
    case IkStatus::Unreachable: code = ErrorCode::Unreachable; break;
    case IkStatus::JointLimitReached: code = ErrorCode::JointLimitReached; break;
    case IkStatus::DidNotConverge: break;
    }
    return Failure(code, result.message);
}

kinematics::IkOptions RuntimeLinearIkOptions()
{
    kinematics::IkOptions options = detail::PathIkOptions();
    // 특이 자세에서 관절을 재배치하는 보조 IK에 완화한 오차를 사용해 반복 횟수를 제한한다.
    options.positionToleranceMeters = 1e-6;
    options.orientationToleranceRadians = 1e-5;
    return options;
}

double TrapezoidDistanceFraction(double timeFraction, double rampTimeFraction)
{
    // 프로파일은 시간의 앞·뒤 rampTimeFraction 비율에서 속도를 선형으로 올리고 내리며, 가운데는 최대 속도를 유지한다.
    // 속도를 적분한 뒤 전체 면적 1-ramp와 비교해 경로 전체가 정확히 0부터 1까지 진행하도록 정규화한다.
    const double ramp = std::clamp(rampTimeFraction, 1e-6, 0.49);
    const double u = std::clamp(timeFraction, 0.0, 1.0);
    double integratedSpeed = 0.0;
    if (u < ramp)
        integratedSpeed = u * u / (2.0 * ramp);
    else if (u <= 1.0 - ramp)
        integratedSpeed = ramp * 0.5 + (u - ramp);
    else
    {
        const double decelerationStart = 1.0 - ramp;
        integratedSpeed = ramp * 0.5 + (1.0 - 2.0 * ramp) +
            ((u - u * u * 0.5) - (decelerationStart - decelerationStart * decelerationStart * 0.5)) / ramp;
    }
    return std::clamp(integratedSpeed / (1.0 - ramp), 0.0, 1.0);
}
} // 익명 네임스페이스

SimRobotController::SimRobotController(const models::RobotSpecification& specification, models::Pose3 tcpInToolFrame)
    : specification_(&specification), inverse_(specification, tcpInToolFrame)
{
    if (specification_->joints == nullptr || specification_->jointCount == 0)
        throw std::invalid_argument("SimRobotController: empty robot specification");
    // 상태는 q=0에서 시작하므로 모든 관절 범위가 0을 포함하고 제한 속도가 양수여야 한다.
    for (std::size_t i = 0; i < specification_->jointCount; ++i)
    {
        const auto& joint = specification_->joints[i];
        // 초기 상태는 q=0. 이를 허용하지 않는 모델은 별도 초기화 계약이 필요하다.
        if (!std::isfinite(joint.minPositionRadians) || !std::isfinite(joint.maxPositionRadians) ||
            joint.minPositionRadians > 0.0 || joint.maxPositionRadians < 0.0 ||
            !std::isfinite(joint.maxVelocityRadiansPerSecond) || joint.maxVelocityRadiansPerSecond <= 0.0)
            throw std::invalid_argument("SimRobotController: invalid limits, zero pose or maximum velocity");
    }
}

void SimRobotController::SetJointPoseCollisionValidator(std::function<bool(const JointVector&)> validator)
{
    collisionValidator_ = std::move(validator);
}

std::vector<JointVector> SimRobotController::BuildIkSeeds(const JointVector& start) const
{
    std::vector<JointVector> seeds{start};
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

    // J1은 팔을 반대쪽으로 돌리고, J3은 팔꿈치를 접거나 펴며, J5는 손목 굽힘 방향을 바꾸는 대표적인 해 분기 관절이다.
    // 각 관절각을 허용 범위의 반대편으로 비춘 조합을 시작점으로 넣어 DLS가 현재 해에만 갇히지 않게 한다.
    constexpr std::array<std::size_t, 3> branchJoints{0, 2, 4};
    for (unsigned mask = 1; mask < (1U << branchJoints.size()); ++mask)
    {
        JointVector candidate = start;
        for (std::size_t bit = 0; bit < branchJoints.size(); ++bit)
        {
            if ((mask & (1U << bit)) == 0 || branchJoints[bit] >= candidate.size())
                continue;
            const auto& joint = specification_->joints[branchJoints[bit]];
            candidate[branchJoints[bit]] = std::clamp(
                joint.minPositionRadians + joint.maxPositionRadians - candidate[branchJoints[bit]],
                joint.minPositionRadians,
                joint.maxPositionRadians);
        }
        addSeed(std::move(candidate));
    }

    // 대체 seed는 현재 자세와 J1·J3·J5를 허용 범위의 반대편으로 반사한 조합만 포함하며, 가능한 IK 해 전체를 열거하지 않는다.
    return seeds;
}

bool SimRobotController::IsJointPathCollisionFree(const JointVector& start, const JointVector& end) const
{
    if (!collisionValidator_)
        return true;

    double maximumJointChange = 0.0;
    for (std::size_t joint = 0; joint < start.size(); ++joint)
        maximumJointChange = std::max(maximumJointChange, std::abs(end[joint] - start[joint]));
    const std::size_t intervals = std::max<std::size_t>(1, static_cast<std::size_t>(std::ceil(maximumJointChange / 0.08)));
    JointVector sample(start.size());
    for (std::size_t step = 1; step <= intervals; ++step)
    {
        const double fraction = static_cast<double>(step) / static_cast<double>(intervals);
        for (std::size_t joint = 0; joint < start.size(); ++joint)
            sample[joint] = start[joint] + (end[joint] - start[joint]) * fraction;
        if (!collisionValidator_(sample))
            return false;
    }
    return true;
}

std::optional<kinematics::IkResult> SimRobotController::SolveCollisionFreeIk(
    const CartesianPose& target,
    const JointVector& start,
    const kinematics::IkOptions& options,
    bool& collisionBlocked,
    kinematics::IkResult& ikFailure)
{
    collisionBlocked = false;
    // 현재 관절각에서 목표에 수렴하면 그 해부터 쓴다. 직전 해를 초기값으로 이어 가므로 경로 중 관절이 다른 가지로 갑자기 바뀌는 일을 줄인다.
    auto preferred = inverse_.SolveSingleSeed(target, start, options);
    if (preferred)
    {
        AlignEquivalentJointAngles(preferred.jointPositionRadians, start, *specification_);
        if (!collisionValidator_ || IsJointPathCollisionFree(start, preferred.jointPositionRadians))
            return preferred;
        collisionBlocked = true;
    }
    else
    {
        ikFailure = preferred;
    }

    // 첫 해가 수렴하지 않거나 충돌 검사에 걸렸을 때만 J1·J3·J5의 대체 자세를 시험한다. 각 대체 해는 관절 허용 범위로 나눈 이동량을 비교해 현재 자세와 가장 가까운 안전한 해를 고른다.
    bool foundAnyIkSolution = false;
    std::optional<kinematics::IkResult> bestSolution;
    double bestNormalizedDistance = std::numeric_limits<double>::infinity();
    for (const auto& seed : BuildIkSeeds(start))
    {
        bool sameAsStart = true;
        for (std::size_t joint = 0; joint < start.size(); ++joint)
            sameAsStart = sameAsStart && std::abs(seed[joint] - start[joint]) <= 1e-8;
        if (sameAsStart)
            continue;

        auto solution = inverse_.SolveSingleSeed(target, seed, options);
        if (!solution)
        {
            ikFailure = std::move(solution);
            continue;
        }
        AlignEquivalentJointAngles(solution.jointPositionRadians, start, *specification_);
        foundAnyIkSolution = true;
        if (collisionValidator_ && !IsJointPathCollisionFree(start, solution.jointPositionRadians))
            continue;

        double normalizedDistance = 0.0;
        for (std::size_t joint = 0; joint < start.size(); ++joint)
        {
            const double range = std::max(1e-12, specification_->joints[joint].maxPositionRadians -
                specification_->joints[joint].minPositionRadians);
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
    collisionBlocked = collisionBlocked || foundAnyIkSolution;
    return std::nullopt;
}

Result SimRobotController::Connect()
{
    // 재연결은 이전 명령과 상태를 이어가지 않고 q=0의 새 논리 세션을 만든다.
    state_ = {};
    state_.jointPositionRadians.assign(specification_->jointCount, 0.0);
    state_.jointVelocityRadiansPerSecond.assign(specification_->jointCount, 0.0);
    state_.mode = RobotMode::Idle;
    state_.valid = true;
    RefreshTcp();

    targetPositionRadians_ = state_.jointPositionRadians;
    linearInterpolationBuffer_.resize(specification_->jointCount);
    velocityScale_ = 1.0;
    accelerationScale_ = 1.0;
    connected_ = true;
    linearPath_.clear();
    return Result::Success();
}

void SimRobotController::Disconnect() noexcept
{
    // 백엔드의 관절 배열은 재사용하되 연결/feedback 유효성을 내리고 속도만 즉시 0으로 만든다.
    connected_ = false;
    state_.mode = RobotMode::Disconnected;
    state_.valid = false;
    state_.tcpPoseValid = false;
    linearPath_.clear();
    std::fill(
        state_.jointVelocityRadiansPerSecond.begin(),
        state_.jointVelocityRadiansPerSecond.end(),
        0.0);
}

bool SimRobotController::IsConnected() const noexcept
{
    return connected_;
}

Result SimRobotController::MoveJoint(const JointMoveCommand& command)
{
    if (!connected_)
        return Failure(ErrorCode::NotConnected, "SimRobotController: not connected");

    if (command.targetPositionRadians.size() != specification_->jointCount)
        return Failure(ErrorCode::InvalidCommand, "SimRobotController: joint count mismatch");

    if (!IsScaleValid(command.velocityScale) || !IsScaleValid(command.accelerationScale))
        return Failure(ErrorCode::InvalidCommand, "SimRobotController: scale must be in (0, 1]");

    // 모든 joint를 먼저 확인한다. 하나라도 범위 밖이면 target이나 기존 진행 상태를 일부만 바꾸지 않는다.
    for (std::size_t i = 0; i < specification_->jointCount; ++i)
    {
        const double target = command.targetPositionRadians[i];
        const auto& joint = specification_->joints[i];
        if (!std::isfinite(target) ||
            target < joint.minPositionRadians ||
            target > joint.maxPositionRadians)
        {
            return Failure(
                ErrorCode::InvalidCommand,
                "SimRobotController: target outside joint limit: " + std::string(joint.name));
        }
    }

    // 명령을 받는 순간 관절각 q를 바꾸지 않는다. 이후 Update 호출이 정해진 속도 안에서 현재 상태를 목표까지 진행시킨다.
    // 움직이는 도중 새 명령이 들어오면 대기열에 쌓지 않고 현재 목표를 새 목표로 바꾼다.
    targetPositionRadians_ = command.targetPositionRadians;
    if (!command.preserveJointTurns)
        AlignEquivalentJointAngles(targetPositionRadians_, state_.jointPositionRadians, *specification_);
    velocityScale_ = command.velocityScale;
    accelerationScale_ = command.accelerationScale;
    linearPath_.clear();
    state_.faultCode = 0;

    bool needsMotion = false;
    for (std::size_t i = 0; i < specification_->jointCount; ++i)
    {
        if (std::abs(targetPositionRadians_[i] - state_.jointPositionRadians[i]) > kPositionEpsilon)
        {
            needsMotion = true;
            break;
        }
    }

    // 이미 현재 자세를 목표로 받은 경우에도 수락은 성공이며 mode만 Idle로 둔다.
    state_.mode = needsMotion ? RobotMode::Moving : RobotMode::Idle;
    if (!needsMotion)
        std::fill(state_.jointVelocityRadiansPerSecond.begin(), state_.jointVelocityRadiansPerSecond.end(), 0.0);
    return Result::Success();
}

Result SimRobotController::MovePose(const CartesianPose& targetInBase, double velocityScale, double accelerationScale)
{
    if (!connected_)
        return Failure(ErrorCode::NotConnected, "SimRobotController: not connected");
    if (!IsScaleValid(velocityScale) || !IsScaleValid(accelerationScale))
        return Failure(ErrorCode::InvalidCommand, "SimRobotController: scale must be in (0, 1]");
    bool collisionBlocked = false;
    kinematics::IkResult ikFailure;
    auto solution = SolveCollisionFreeIk(targetInBase, state_.jointPositionRadians, {}, collisionBlocked, ikFailure);
    if (!solution)
        return collisionBlocked ?
            Failure(ErrorCode::EnvironmentContact, "SimRobotController: no collision-free IK solution or joint path") :
            IkFailure(ikFailure);
    return MoveJoint({solution->jointPositionRadians, velocityScale, accelerationScale});
}

Result SimRobotController::MoveLinear(const LinearMoveCommand& command)
{
    LinearPathMoveCommand path;
    path.targetPoses.push_back(command.targetPose);
    path.maxLinearVelocityMetersPerSecond = command.maxLinearVelocityMetersPerSecond;
    path.maxAngularVelocityRadiansPerSecond = command.maxAngularVelocityRadiansPerSecond;
    path.maxLinearAccelerationMetersPerSecondSquared = command.maxLinearAccelerationMetersPerSecondSquared;
    path.maxAngularAccelerationRadiansPerSecondSquared = command.maxAngularAccelerationRadiansPerSecondSquared;
    return MoveLinearPath(path);
}

Result SimRobotController::MoveLinearPath(const LinearPathMoveCommand& command)
{
    if (!connected_)
        return Failure(ErrorCode::NotConnected, "SimRobotController: not connected");
    if (command.targetPoses.empty() ||
        !std::isfinite(command.maxLinearVelocityMetersPerSecond) || command.maxLinearVelocityMetersPerSecond <= 0.0 ||
        !std::isfinite(command.maxAngularVelocityRadiansPerSecond) || command.maxAngularVelocityRadiansPerSecond <= 0.0 ||
        !std::isfinite(command.maxLinearAccelerationMetersPerSecondSquared) || command.maxLinearAccelerationMetersPerSecondSquared <= 0.0 ||
        !std::isfinite(command.maxAngularAccelerationRadiansPerSecondSquared) || command.maxAngularAccelerationRadiansPerSecondSquared <= 0.0)
        return Failure(ErrorCode::InvalidCommand, "SimRobotController: invalid linear path or motion limits");
    if (!specification_->hasToolFrame)
        return Failure(ErrorCode::Unsupported, "SimRobotController: missing ToolFrame for TCP motion");

    detail::LinearPathPlan plan;
    const auto result = detail::LinearPathPlanner::Build(
        command,
        *specification_,
        state_.jointPositionRadians,
        inverse_.EvaluateTcp(state_.jointPositionRadians),
        [this](const CartesianPose& target, const JointVector& start, const kinematics::IkOptions& options,
            bool& collisionBlocked, kinematics::IkResult& ikFailure)
        {
            return SolveCollisionFreeIk(target, start, options, collisionBlocked, ikFailure);
        },
        [this](const CartesianPose& target, const JointVector& start)
        {
            // 표본 상한 초과와 도달 불가를 구분하는 끝점 검사는 기존 동작처럼 충돌 필터를 적용하지 않는다.
            return inverse_.Solve(target, start);
        },
        plan);
    if (!result)
        return result;
    if (!plan.hasMotion)
        return MoveJoint({state_.jointPositionRadians, 1.0, 1.0});

    auto& candidate = plan.points;
    const double plannedLinearVelocity = plan.plannedLinearVelocity;
    const double plannedAngularVelocity = plan.plannedAngularVelocity;
    linearVelocityLimit_ = command.maxLinearVelocityMetersPerSecond;
    angularVelocityLimit_ = command.maxAngularVelocityRadiansPerSecond;
    linearAccelerationLimit_ = command.maxLinearAccelerationMetersPerSecondSquared;
    angularAccelerationLimit_ = command.maxAngularAccelerationRadiansPerSecondSquared;
    linearPlannedDurationSeconds_ = 0.0;
    for (std::size_t i = 1; i < candidate.size(); ++i)
        linearPlannedDurationSeconds_ += candidate[i].durationSeconds;
    // 모든 직선 구간의 계획 시간을 합쳐 가속·감속을 한 번만 적용한다. 웨이포인트마다 명령을 다시 내리면 매 구간마다 멈췄다가 재가속한다.
    const double accelerationTime = std::max(
        plannedLinearVelocity / linearAccelerationLimit_,
        plannedAngularVelocity / angularAccelerationLimit_);
    linearProfileRampFraction_ = accelerationTime /
        (linearPlannedDurationSeconds_ + accelerationTime);
    linearProfileRampFraction_ = std::clamp(linearProfileRampFraction_, 0.005, 0.45);
    linearProfileDurationSeconds_ = linearPlannedDurationSeconds_ / (1.0 - linearProfileRampFraction_);
    linearProfileElapsedSeconds_ = 0.0;
    targetPositionRadians_ = candidate.back().joints;
    linearPath_ = std::move(candidate);
    linearSegment_ = 1;
    linearSegmentFraction_ = 0.0;
    velocityScale_ = 1.0;
    accelerationScale_ = 1.0;
    state_.faultCode = 0;
    state_.mode = RobotMode::Moving;
    return Result::Success();
}

void SimRobotController::RefreshTcp()
{
    state_.tcpPoseValid = specification_->hasToolFrame && state_.valid;
    if (state_.tcpPoseValid)
        state_.tcpPose = inverse_.EvaluateTcp(state_.jointPositionRadians);
}

Result SimRobotController::Stop()
{
    if (!connected_)
        return Failure(ErrorCode::NotConnected, "SimRobotController: not connected");

    // 현재 q를 새 목표로 고정해 이후 Update가 남은 동작을 재개하지 않게 한다.
    targetPositionRadians_ = state_.jointPositionRadians;
    linearPath_.clear();
    linearProfileElapsedSeconds_ = 0.0;
    std::fill(
        state_.jointVelocityRadiansPerSecond.begin(),
        state_.jointVelocityRadiansPerSecond.end(),
        0.0);
    state_.mode = RobotMode::Stopped;
    return Result::Success();
}

RobotState SimRobotController::GetState() const
{
    // vector까지 값으로 복사하므로 호출자는 다음 Update와 무관한 snapshot을 받는다.
    return state_;
}

const RobotState& SimRobotController::GetStateView() const noexcept
{
    return state_;
}

void SimRobotController::Update(double dtSeconds)
{
    // FixedControlLoop 같은 호출자가 정한 tick마다 q/dq와 모델 FK의 TCP를 갱신한다. 화면 Entity와 물리 Body 반영은 별도 계층의 책임이다.
    if (!connected_ || state_.mode != RobotMode::Moving ||
        !std::isfinite(dtSeconds) || dtSeconds <= 0.0)
    {
        return;
    }

    if (!linearPath_.empty())
    {
        UpdateLinear(dtSeconds);
        return;
    }

    bool allReached = true;

    for (std::size_t i = 0; i < specification_->jointCount; ++i)
    {
        const auto& joint = specification_->joints[i];
        const double delta = targetPositionRadians_[i] - state_.jointPositionRadians[i];

        if (std::abs(delta) <= kPositionEpsilon)
        {
            state_.jointPositionRadians[i] = targetPositionRadians_[i];
            state_.jointVelocityRadiansPerSecond[i] = 0.0;
            continue;
        }

        // 각 관절은 지정된 최대 각속도를 넘지 않게 목표를 향해 움직인다. 속도 변화에 대한 가속도 제한이나 부드러운 ramp는 적용하지 않는다.
        // maxStep은 이번 간격에 허용되는 각도 [rad]. clamp로 큰 dt에서도 목표를 지나치지 않는다.
        const double maxStep = joint.maxVelocityRadiansPerSecond * velocityScale_ * dtSeconds;
        if (maxStep <= 0.0)
        {
            state_.jointVelocityRadiansPerSecond[i] = 0.0;
            allReached = false;
            continue;
        }

        const double step = std::clamp(delta, -maxStep, maxStep);
        state_.jointPositionRadians[i] += step;
        state_.jointVelocityRadiansPerSecond[i] = step / dtSeconds;

        if (std::abs(targetPositionRadians_[i] - state_.jointPositionRadians[i]) > kPositionEpsilon)
        {
            allReached = false;
        }
        else
        {
            state_.jointPositionRadians[i] = targetPositionRadians_[i];
        }
    }

    // MoveJoint 명령의 accelerationScale은 아직 쓰지 않고, 각 관절에 모델의 최대 각속도를 즉시 적용한다.
    (void)accelerationScale_;

    if (allReached)
    {
        std::fill(
            state_.jointVelocityRadiansPerSecond.begin(),
            state_.jointVelocityRadiansPerSecond.end(),
            0.0);
        state_.mode = RobotMode::Idle;
    }
    RefreshTcp();
}

void SimRobotController::UpdateLinear(double dtSeconds)
{
    using namespace kinematics::detail;
    // 이번 tick 시작 관절각은 별도 임시 vector 대신, 끝에서 새 속도로 덮어쓸 속도 배열에 잠시 보관한다.
    std::copy(state_.jointPositionRadians.begin(), state_.jointPositionRadians.end(),
        state_.jointVelocityRadiansPerSecond.begin());
    double currentPathTime = 0.0;
    for (std::size_t i = 1; i < linearSegment_; ++i)
        currentPathTime += linearPath_[i].durationSeconds;
    if (linearSegment_ < linearPath_.size())
        currentPathTime += linearSegmentFraction_ * linearPath_[linearSegment_].durationSeconds;
    linearProfileElapsedSeconds_ = std::min(linearProfileDurationSeconds_, linearProfileElapsedSeconds_ + dtSeconds);
    const double profileEnd = linearProfileElapsedSeconds_ / linearProfileDurationSeconds_;
    const double desiredPathTime = linearPlannedDurationSeconds_ *
        TrapezoidDistanceFraction(profileEnd, linearProfileRampFraction_);
    double pathTimeBudget = std::max(0.0, desiredPathTime - currentPathTime);
    double remaining = dtSeconds;
    bool failed = false;
    ErrorCode failureCode = ErrorCode::IkDidNotConverge;
    JointVector& interpolatedJoints = linearInterpolationBuffer_;
    while (remaining > 0.0 && pathTimeBudget > 0.0 && linearSegment_ < linearPath_.size())
    {
        const auto& begin = linearPath_[linearSegment_ - 1];
        const auto& end = linearPath_[linearSegment_];
        const double segmentRemaining = (1.0 - linearSegmentFraction_) * end.durationSeconds;
        const double available = remaining;
        double fractionStep = std::min(pathTimeBudget, segmentRemaining) / end.durationSeconds;
        double usedSeconds = 0.0;
        // 유한한 양수 시간도 표현 가능한 경로 비율보다 작을 수 있다. 나눗셈의 0 분모나 진행 없는 반복 대신 현재 자세를 유지하고 다음 tick을 기다린다.
        if (!std::isfinite(available) || available <= 0.0 || fractionStep <= 0.0 ||
            linearSegmentFraction_ + fractionStep == linearSegmentFraction_)
            break;
        const Pose3 currentTcp = FromCartesian(inverse_.EvaluateTcp(state_.jointPositionRadians));
        bool accepted = false;
        for (int attempt = 0; attempt < 16; ++attempt)
        {
            const double nextSegmentFraction = std::min(1.0, linearSegmentFraction_ + fractionStep);
            if (nextSegmentFraction == linearSegmentFraction_)
            {
                accepted = true;
                usedSeconds = available;
                break;
            }
            // 명령을 받을 때 직선 TCP 목표마다 IK와 충돌 검사를 끝낸 경로 표본 사이를 보간해 매 tick IK 반복을 없앤다.
            for (std::size_t joint = 0; joint < specification_->jointCount; ++joint)
                interpolatedJoints[joint] = begin.joints[joint] +
                    (end.joints[joint] - begin.joints[joint]) * nextSegmentFraction;
            double requiredSeconds = 0.0;
            for (std::size_t joint = 0; joint < specification_->jointCount; ++joint)
                requiredSeconds = std::max(requiredSeconds, RequiredTimeForVelocity(
                    interpolatedJoints[joint] - state_.jointPositionRadians[joint],
                    specification_->joints[joint].maxVelocityRadiansPerSecond));
            const Pose3 nextTcp = FromCartesian(inverse_.EvaluateTcp(interpolatedJoints));
            requiredSeconds = std::max(requiredSeconds,
                RequiredTimeForVelocity(
                    Length(Subtract(nextTcp.positionMeters, currentTcp.positionMeters)), linearVelocityLimit_));
            requiredSeconds = std::max(requiredSeconds,
                RequiredTimeForVelocity(
                    Length(RotationError(nextTcp.rotation, currentTcp.rotation)), angularVelocityLimit_));
            const double ratio = std::max(1.0, requiredSeconds / available);
            if (ratio <= 1.0 + 1e-8)
            {
                accepted = true;
                state_.jointPositionRadians = interpolatedJoints;
                linearSegmentFraction_ = nextSegmentFraction;
                // 표본 끝에 남은 명목 시간이 아주 작아도 IK 허용 오차를 넘는 보정에는 실제 시간이 필요하다.
                // 남은 실제 tick 전체를 속도 예산으로 쓰고, 명목 진행 시간과 실제 변화에 필요한 시간 중 큰 값을 소비해 같은 시간을 다음 구간에 중복 사용하지 않는다.
                usedSeconds = std::min(available, std::max(fractionStep * end.durationSeconds, requiredSeconds));
                pathTimeBudget -= fractionStep * end.durationSeconds;
                break;
            }
            // 실제 관절 변화가 속도 상한을 넘으면 이번 tick의 경로 진행 비율을 줄여 다시 확인한다.
            fractionStep *= 0.8 / ratio;
        }
        if (!accepted)
        {
            // 특이 자세에서는 TCP를 거의 움직이지 않고 손목 관절끼리 반대 방향으로 돌려야 다음 직선 자세를 만들 수 있다.
            // 경로 표본의 관절각 방향을 참고하되 실제 TCP와 모든 속도 상한을 다시 확인한 작은 보조 이동만 허용한다.
            if (!ReorientForLinear(end.joints, available))
            {
                failed = true;
                break;
            }
            remaining = 0.0;
            continue;
        }
        remaining = usedSeconds >= remaining ? 0.0 : remaining - usedSeconds;
        if (linearSegmentFraction_ >= 1.0 - 1e-12)
        {
            linearSegmentFraction_ = 0.0;
            ++linearSegment_;
        }
        else
        {
            // 속도 때문에 경로 진행을 줄였으면 이번 시간은 모두 소비했다. 남은 경로 시간은 다음 Update가 이어서 처리한다.
            remaining = 0.0;
        }
    }
    if (failed)
    {
        targetPositionRadians_ = state_.jointPositionRadians;
        linearPath_.clear();
        state_.mode = RobotMode::Fault;
        state_.faultCode = static_cast<std::uint32_t>(failureCode);
    }
    else if (linearSegment_ >= linearPath_.size())
    {
        linearPath_.clear();
        state_.mode = RobotMode::Idle;
    }
    for (std::size_t joint = 0; joint < specification_->jointCount; ++joint)
    {
        const double previousPosition = state_.jointVelocityRadiansPerSecond[joint];
        state_.jointVelocityRadiansPerSecond[joint] = state_.mode == RobotMode::Moving ?
            (state_.jointPositionRadians[joint] - previousPosition) / dtSeconds : 0.0;
    }
    RefreshTcp();
}

bool SimRobotController::ReorientForLinear(const JointVector& plannedJoints, double availableSeconds)
{
    using namespace kinematics::detail;
    const JointVector current = state_.jointPositionRadians;
    const Pose3 currentTcp = FromCartesian(inverse_.EvaluateTcp(current));
    const auto& begin = linearPath_[linearSegment_ - 1];
    const auto& end = linearPath_[linearSegment_];
    const Pose3 pathTcp = Interpolate(FromCartesian(begin.tcpPose), FromCartesian(end.tcpPose), linearSegmentFraction_);
    const auto options = RuntimeLinearIkOptions();
    double attraction = 1.0;
    double previousDistanceSquared = 0.0;
    for (std::size_t i = 0; i < current.size(); ++i)
    {
        const double delta = plannedJoints[i] - current[i];
        previousDistanceSquared += delta * delta;
        if (std::abs(delta) > 0.0)
            attraction = std::min(attraction, specification_->joints[i].maxVelocityRadiansPerSecond * availableSeconds / std::abs(delta));
    }
    // 후보 시작각을 다음 표본 방향으로 작은 비율만 옮기고 현재 TCP 자세의 IK를 다시 푼다.
    // IK가 위치·방향을 되돌리는 동안 남는 관절 변화는 TCP를 거의 바꾸지 않는 여유 방향이며 nullspace라고 부른다.
    // 일반 모델에서 단순 관절 보간이 TCP를 고정한다고 가정하지 않으므로 후보마다 FK/속도 검사를 거친다.
    for (int attempt = 0; attempt < 12; ++attempt)
    {
        JointVector seed = current;
        for (std::size_t i = 0; i < current.size(); ++i)
            seed[i] += (plannedJoints[i] - current[i]) * attraction;
        // 현재 FK 값 대신 명목 경로의 같은 진행률 자세로 복원해 반복 보조 이동의 작은 IK 오차가 직선 밖으로 누적되지 않게 한다.
        const auto solution = inverse_.Solve(ToCartesian(pathTcp), seed, options);
        if (solution)
        {
            double ratio = 1.0;
            double nextDistanceSquared = 0.0;
            double movement = 0.0;
            for (std::size_t i = 0; i < current.size(); ++i)
            {
                const double change = solution.jointPositionRadians[i] - current[i];
                ratio = std::max(ratio, VelocityRatio(
                    change, availableSeconds, specification_->joints[i].maxVelocityRadiansPerSecond));
                movement = std::max(movement, std::abs(change));
                const double residual = plannedJoints[i] - solution.jointPositionRadians[i];
                nextDistanceSquared += residual * residual;
            }
            const Pose3 nextTcp = FromCartesian(inverse_.EvaluateTcp(solution.jointPositionRadians));
            ratio = std::max(ratio, VelocityRatio(
                Length(Subtract(nextTcp.positionMeters, currentTcp.positionMeters)), availableSeconds, linearVelocityLimit_));
            ratio = std::max(ratio, VelocityRatio(
                Length(RotationError(nextTcp.rotation, currentTcp.rotation)), availableSeconds, angularVelocityLimit_));
            if (ratio <= 1.0 + 1e-8 && movement > 1e-12 && nextDistanceSquared < previousDistanceSquared)
            {
                state_.jointPositionRadians = solution.jointPositionRadians;
                return true;
            }
        }
        attraction *= 0.5;
    }
    return false;
}

const models::RobotSpecification& SimRobotController::GetSpecification() const noexcept
{
    // 생성 시 받은 사양을 복사하지 않고 참조한다. 따라서 사양 데이터를 가진 저장소는 이 Controller보다 오래 살아 있어야 한다.
    return *specification_;
}

bool SimRobotController::RestoreCollisionSafeState(const JointVector& safePositionRadians)
{
    if (safePositionRadians.size() != specification_->jointCount)
        return false;
    for (std::size_t joint = 0; joint < safePositionRadians.size(); ++joint)
        if (!std::isfinite(safePositionRadians[joint]) || safePositionRadians[joint] < specification_->joints[joint].minPositionRadians ||
            safePositionRadians[joint] > specification_->joints[joint].maxPositionRadians)
            return false;

    state_.jointPositionRadians = safePositionRadians;
    std::fill(state_.jointVelocityRadiansPerSecond.begin(), state_.jointVelocityRadiansPerSecond.end(), 0.0);
    targetPositionRadians_ = safePositionRadians;
    linearPath_.clear();
    // unsafe tick은 직전 안전 자세로 되돌린 뒤 멈춘다. Fault로 고정하면 호출자가 위쪽 안전 자세로 물러나는 명령도 보낼 수 없다.
    state_.mode = RobotMode::Idle;
    state_.faultCode = static_cast<std::uint32_t>(ErrorCode::EnvironmentContact);
    RefreshTcp();
    return true;
}

} // grasplink::robotics::backends::simulation 네임스페이스
