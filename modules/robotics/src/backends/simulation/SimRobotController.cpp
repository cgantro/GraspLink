#include "robotics/backends/simulation/SimRobotController.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace grasplink::robotics::backends::simulation
{
namespace
{
constexpr double kPositionEpsilon = 1e-8;

Result Failure(ErrorCode code, std::string message)
{
    return {code, std::move(message)};
}

bool IsScaleValid(double value)
{
    return std::isfinite(value) && value > 0.0 && value <= 1.0;
}
} // namespace

SimRobotController::SimRobotController(const models::RobotSpecification& specification)
    : specification_(&specification)
{
    if (specification_->joints == nullptr || specification_->jointCount == 0)
        throw std::invalid_argument("SimRobotController: empty robot specification");
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

Result SimRobotController::Connect()
{
    state_ = {};
    state_.jointPositionRadians.assign(specification_->jointCount, 0.0);
    state_.jointVelocityRadiansPerSecond.assign(specification_->jointCount, 0.0);
    state_.mode = RobotMode::Idle;
    state_.valid = true;
    // TCP pose는 이 Controller가 채우지 않는다. RobotKinematics가 관절 상태에서 별도 FK 결과를 만든다.
    state_.tcpPoseValid = false;

    targetPositionRadians_ = state_.jointPositionRadians;
    velocityScale_ = 1.0;
    accelerationScale_ = 1.0;
    connected_ = true;
    return Result::Success();
}

void SimRobotController::Disconnect() noexcept
{
    connected_ = false;
    state_.mode = RobotMode::Disconnected;
    state_.valid = false;
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

    // Moving 중 새 명령도 허용하며 진행 중이던 target을 교체한다.
    targetPositionRadians_ = command.targetPositionRadians;
    velocityScale_ = command.velocityScale;
    accelerationScale_ = command.accelerationScale;

    bool needsMotion = false;
    for (std::size_t i = 0; i < specification_->jointCount; ++i)
    {
        if (std::abs(targetPositionRadians_[i] - state_.jointPositionRadians[i]) > kPositionEpsilon)
        {
            needsMotion = true;
            break;
        }
    }

    state_.mode = needsMotion ? RobotMode::Moving : RobotMode::Idle;
    return Result::Success();
}

Result SimRobotController::MoveLinear(const LinearMoveCommand&)
{
    if (!connected_)
        return Failure(ErrorCode::NotConnected, "SimRobotController: not connected");

    return Failure(
        ErrorCode::Unsupported,
        "SimRobotController: MoveLinear requires an IK solver and trajectory execution");
}

Result SimRobotController::Stop()
{
    if (!connected_)
        return Failure(ErrorCode::NotConnected, "SimRobotController: not connected");

    targetPositionRadians_ = state_.jointPositionRadians;
    std::fill(
        state_.jointVelocityRadiansPerSecond.begin(),
        state_.jointVelocityRadiansPerSecond.end(),
        0.0);
    state_.mode = RobotMode::Stopped;
    return Result::Success();
}

RobotState SimRobotController::GetState() const
{
    return state_;
}

void SimRobotController::Update(double dtSeconds)
{
    if (!connected_ || state_.mode != RobotMode::Moving ||
        !std::isfinite(dtSeconds) || dtSeconds <= 0.0)
    {
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

        // 속도 상한 안에서 target을 직접 따라간다. 가속도 제한이나 ramp는 적용하지 않는다.
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

    // accelerationScale_는 계약 보존용이다. Simulation은 가속도 제한을 계산하지 않는다.
    (void)accelerationScale_;

    if (allReached)
    {
        std::fill(
            state_.jointVelocityRadiansPerSecond.begin(),
            state_.jointVelocityRadiansPerSecond.end(),
            0.0);
        state_.mode = RobotMode::Idle;
    }
}

const models::RobotSpecification& SimRobotController::GetSpecification() const noexcept
{
    return *specification_;
}

} // namespace grasplink::robotics::backends::simulation
