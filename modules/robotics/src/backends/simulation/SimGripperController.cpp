#include "robotics/backends/simulation/SimGripperController.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace grasplink::robotics::backends::simulation
{
namespace
{
Result Failure(ErrorCode code, const char* message)
{
    return {code, message};
}

bool IsFinitePositive(double value)
{
    return std::isfinite(value) && value > 0.0;
}
} // namespace

SimGripperController::SimGripperController(
    const models::GripperSpecification& specification,
    SimGripperMotionSettings settings)
    : specification_(&specification), settings_(settings)
{
    if (specification_->joints == nullptr || specification_->jointCount == 0 ||
        specification_->positionRequestMin >= specification_->positionRequestMax ||
        specification_->speedRequestMin >= specification_->speedRequestMax ||
        specification_->forceRequestMin > specification_->forceRequestMax ||
        !IsFinitePositive(specification_->nominalMasterClosedRadians))
    {
        throw std::invalid_argument("SimGripperController: invalid gripper specification");
    }

    if (!IsFinitePositive(settings_.minimumMasterVelocityRadiansPerSecond) ||
        !IsFinitePositive(settings_.maximumMasterVelocityRadiansPerSecond) ||
        settings_.minimumMasterVelocityRadiansPerSecond > settings_.maximumMasterVelocityRadiansPerSecond)
    {
        throw std::invalid_argument("SimGripperController: invalid master velocity settings");
    }
}

Result SimGripperController::Connect()
{
    if (connected_)
        return Result::Success();

    state_ = {};
    state_.mode = GripperMode::Inactive;
    state_.objectStatus = GripperObjectStatus::AtRequestedPosition;
    state_.requestedPositionEcho = specification_->positionRequestMin;
    state_.actualPosition = specification_->positionRequestMin;
    state_.currentRaw = 0;
    state_.currentValid = false;
    state_.closureFraction = 0.0;
    state_.closureFractionValid = true;
    state_.valid = true;
    closureFraction_ = 0.0;
    targetClosureFraction_ = 0.0;
    masterVelocityRadiansPerSecond_ = 0.0;
    connected_ = true;
    return Result::Success();
}

void SimGripperController::Disconnect() noexcept
{
    connected_ = false;
    state_ = {};
    state_.mode = GripperMode::Disconnected;
    state_.valid = false;
    state_.closureFractionValid = false;
    closureFraction_ = 0.0;
    targetClosureFraction_ = 0.0;
    masterVelocityRadiansPerSecond_ = 0.0;
}

bool SimGripperController::IsConnected() const noexcept
{
    return connected_;
}

Result SimGripperController::Activate()
{
    if (!connected_)
        return Failure(ErrorCode::NotConnected, "SimGripperController: not connected");

    if (state_.activated)
        return Result::Success();

    state_.activated = true;
    state_.goToActive = false;
    state_.mode = GripperMode::Idle;
    state_.objectStatus = IsAtTarget()
        ? GripperObjectStatus::AtRequestedPosition
        : GripperObjectStatus::Moving;
    return Result::Success();
}

Result SimGripperController::Reset()
{
    if (!connected_)
        return Failure(ErrorCode::NotConnected, "SimGripperController: not connected");

    const bool reachedRequestedPosition = IsAtTarget();
    masterVelocityRadiansPerSecond_ = 0.0;
    state_.activated = false;
    state_.goToActive = false;
    state_.mode = GripperMode::Inactive;
    state_.objectStatus = reachedRequestedPosition
        ? GripperObjectStatus::AtRequestedPosition
        : GripperObjectStatus::Moving;
    return Result::Success();
}

Result SimGripperController::Command(const GripperCommand& command)
{
    if (!connected_)
        return Failure(ErrorCode::NotConnected, "SimGripperController: not connected");
    if (!state_.activated)
        return Failure(ErrorCode::Busy, "SimGripperController: not activated");

    // 모든 raw 범위를 먼저 확인한다. 거부된 명령은 snapshot과 현재 목표를 그대로 둔다.
    if (command.positionRequest < specification_->positionRequestMin ||
        command.positionRequest > specification_->positionRequestMax ||
        command.speedRequest < specification_->speedRequestMin ||
        command.speedRequest > specification_->speedRequestMax ||
        command.forceRequest < specification_->forceRequestMin ||
        command.forceRequest > specification_->forceRequestMax)
    {
        return Failure(ErrorCode::InvalidCommand, "SimGripperController: request outside specification range");
    }

    const double positionRange = static_cast<double>(
        specification_->positionRequestMax - specification_->positionRequestMin);
    targetClosureFraction_ = static_cast<double>(
        command.positionRequest - specification_->positionRequestMin) / positionRange;

    const double speedRange = static_cast<double>(
        specification_->speedRequestMax - specification_->speedRequestMin);
    const double speedFraction = static_cast<double>(
        command.speedRequest - specification_->speedRequestMin) / speedRange;
    masterVelocityRadiansPerSecond_ = settings_.minimumMasterVelocityRadiansPerSecond +
        speedFraction * (settings_.maximumMasterVelocityRadiansPerSecond -
            settings_.minimumMasterVelocityRadiansPerSecond);

    state_.requestedPositionEcho = command.positionRequest;
    state_.goToActive = !IsAtTarget();
    state_.mode = state_.goToActive ? GripperMode::Moving : GripperMode::Idle;
    state_.objectStatus = state_.goToActive
        ? GripperObjectStatus::Moving
        : GripperObjectStatus::AtRequestedPosition;
    return Result::Success();
}

Result SimGripperController::Stop()
{
    if (!connected_)
        return Failure(ErrorCode::NotConnected, "SimGripperController: not connected");

    const bool reachedRequestedPosition = IsAtTarget();
    masterVelocityRadiansPerSecond_ = 0.0;
    state_.goToActive = false;
    state_.mode = GripperMode::Stopped;
    state_.objectStatus = reachedRequestedPosition
        ? GripperObjectStatus::AtRequestedPosition
        : GripperObjectStatus::Moving;
    return Result::Success();
}

GripperState SimGripperController::GetState() const
{
    return state_;
}

void SimGripperController::Update(double dtSeconds)
{
    if (!connected_ || !state_.activated || state_.mode != GripperMode::Moving ||
        !std::isfinite(dtSeconds) || dtSeconds <= 0.0)
    {
        return;
    }

    // 연속 위치는 master 각속도를 nominal closed 각도로 나눠 진행한다. raw 정수는 관절 계산에 재입력하지 않는다.
    const double maximumClosureStep = masterVelocityRadiansPerSecond_ * dtSeconds /
        specification_->nominalMasterClosedRadians;
    const double remaining = targetClosureFraction_ - closureFraction_;
    if (std::abs(remaining) <= maximumClosureStep)
        closureFraction_ = targetClosureFraction_;
    else
        closureFraction_ += std::copysign(maximumClosureStep, remaining);

    state_.closureFraction = closureFraction_;
    UpdateRawPosition();

    if (IsAtTarget())
    {
        state_.closureFraction = targetClosureFraction_;
        closureFraction_ = targetClosureFraction_;
        UpdateRawPosition();
        state_.goToActive = false;
        state_.mode = GripperMode::Idle;
        state_.objectStatus = GripperObjectStatus::AtRequestedPosition;
        masterVelocityRadiansPerSecond_ = 0.0;
    }
}

bool SimGripperController::IsAtTarget() const noexcept
{
    return closureFraction_ == targetClosureFraction_;
}

void SimGripperController::UpdateRawPosition() noexcept
{
    const double raw = specification_->positionRequestMin + closureFraction_ *
        static_cast<double>(specification_->positionRequestMax - specification_->positionRequestMin);
    const long rounded = std::lround(raw);
    const long bounded = std::clamp(rounded,
        static_cast<long>(specification_->positionRequestMin),
        static_cast<long>(specification_->positionRequestMax));
    state_.actualPosition = static_cast<std::uint8_t>(bounded);
}

} // namespace grasplink::robotics::backends::simulation
