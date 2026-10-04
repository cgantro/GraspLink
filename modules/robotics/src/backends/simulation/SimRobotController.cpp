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
/**
 * @brief 목표 도달 판정에 사용하는 관절 위치 허용 오차 [rad].
 *
 * 부동소수점 연산으로 target과 현재값이 정확히 같은 bit pattern이 되지 않을 수 있으므로
 * 이 값 이하의 차이는 목표 도달로 간주하고 target 값으로 snap한다.
 */
constexpr double kPositionEpsilon = 1e-8;

/**
 * @brief 실패 Result 생성 시 message 문자열을 불필요하게 복사하지 않도록 묶은 내부 helper.
 * @param code 공통 ErrorCode.
 * @param message 호출자에게 전달할 진단 문자열.
 * @return 실패 상태 Result.
 */
Result Failure(ErrorCode code, std::string message)
{
    return {code, std::move(message)};
}

/**
 * @brief velocityScale/accelerationScale이 현재 command contract의 유효 범위인지 검사한다.
 * @param value 검사할 무차원 scale.
 * @return finite이고 (0,1] 범위면 true.
 */
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
}

Result SimRobotController::Connect()
{
    /*
     * Simulation의 logical zero pose에서 시작한다.
     * 실제 HCR hardware의 encoder zero/home offset과 동일하다는 의미가 아니라
     * 현재 simulation model의 q={0,...,0} 상태다. Zero Offset 계층은 향후 별도 구현한다.
     */
    state_ = {};
    state_.jointPositionRadians.assign(specification_->jointCount, 0.0);
    state_.jointVelocityRadiansPerSecond.assign(specification_->jointCount, 0.0);
    state_.mode = RobotMode::Idle;
    state_.valid = true;
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

    /* RobotSpecification의 joint order/count가 모든 JointVector의 contract다. */
    if (command.targetPositionRadians.size() != specification_->jointCount)
        return Failure(ErrorCode::InvalidCommand, "SimRobotController: joint count mismatch");

    if (!IsScaleValid(command.velocityScale) || !IsScaleValid(command.accelerationScale))
        return Failure(ErrorCode::InvalidCommand, "SimRobotController: scale must be in (0, 1]");

    /*
     * 모델별 angle limit [rad]을 command 수락 시점에 검사한다.
     * limit을 넘어온 값을 조용히 clamp하지 않고 command 자체를 InvalidCommand로 거절한다.
     */
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

    /* Cartesian target을 joint target으로 바꿀 IK/trajectory 계층이 아직 없으므로 의도적으로 거절한다. */
    return Failure(
        ErrorCode::Unsupported,
        "SimRobotController: MoveLinear requires the FK/IK layer and is not wired yet");
}

Result SimRobotController::Stop()
{
    if (!connected_)
        return Failure(ErrorCode::NotConnected, "SimRobotController: not connected");

    /* 현재 위치를 새 target으로 만들어 다음 Update부터 추가 이동이 일어나지 않게 한다. */
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
    /* 값 복사 snapshot을 반환해 외부 코드가 Controller 내부 state를 직접 수정하지 못하게 한다. */
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

        // 목표까지 남은 signed angular displacement [rad].
        const double delta = targetPositionRadians_[i] - state_.jointPositionRadians[i];

        if (std::abs(delta) <= kPositionEpsilon)
        {
            state_.jointPositionRadians[i] = targetPositionRadians_[i];
            state_.jointVelocityRadiansPerSecond[i] = 0.0;
            continue;
        }

        /*
         * JointSpecification의 max velocity [rad/s]를 현재 command의 무차원 scale과 dt [s]에 곱해
         * 이번 simulation step에서 이동 가능한 최대 각도 [rad]를 구한다.
         *
         * maxStep[rad] = maxVelocity[rad/s] * velocityScale[-] * dt[s]
         */
        const double maxStep = joint.maxVelocityRadiansPerSecond * velocityScale_ * dtSeconds;
        if (maxStep <= 0.0)
        {
            state_.jointVelocityRadiansPerSecond[i] = 0.0;
            allReached = false;
            continue;
        }

        // 목표를 지나치지 않도록 signed delta를 이번 step의 허용 범위로 제한한다.
        const double step = std::clamp(delta, -maxStep, maxStep);
        state_.jointPositionRadians[i] += step;

        // 실제 이번 step 이동량으로 feedback velocity [rad/s]를 계산한다.
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

    /*
     * accelerationScale_은 command contract에 보존하지만 현재 HCR specification에 검증된 max acceleration 값이 없다.
     * 임의 상수를 제조사 사양처럼 만들지 않기 위해 acceleration limiting은 의도적으로 아직 적용하지 않는다.
     */
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
