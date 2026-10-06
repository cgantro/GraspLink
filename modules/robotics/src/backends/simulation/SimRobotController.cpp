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
    // 잘못된 명령 입력은 예외로 던지지 않고 다른 Controller 구현과 같은 Result 오류 값으로 반환한다.
    return {code, std::move(message)};
}

bool IsScaleValid(double value)
{
    return std::isfinite(value) && value > 0.0 && value <= 1.0;
}
} // 익명 네임스페이스

SimRobotController::SimRobotController(const models::RobotSpecification& specification)
    : specification_(&specification)
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

Result SimRobotController::Connect()
{
    // 재연결은 이전 명령과 상태를 이어가지 않고 q=0의 새 논리 세션을 만든다.
    state_ = {};
    state_.jointPositionRadians.assign(specification_->jointCount, 0.0);
    state_.jointVelocityRadiansPerSecond.assign(specification_->jointCount, 0.0);
    state_.mode = RobotMode::Idle;
    state_.valid = true;
    // 이 시뮬레이션 Controller는 TCP 자세를 feedback으로 제공하지 않는다. 호출자가 관절 상태를 RobotKinematics에 전달해 FK로 따로 계산해야 한다.
    state_.tcpPoseValid = false;

    targetPositionRadians_ = state_.jointPositionRadians;
    velocityScale_ = 1.0;
    accelerationScale_ = 1.0;
    connected_ = true;
    return Result::Success();
}

void SimRobotController::Disconnect() noexcept
{
    // 백엔드의 관절 배열은 재사용하되 연결/feedback 유효성을 내리고 속도만 즉시 0으로 만든다.
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

    // 이미 현재 자세를 목표로 받은 경우에도 수락은 성공이며 mode만 Idle로 둔다.
    state_.mode = needsMotion ? RobotMode::Moving : RobotMode::Idle;
    return Result::Success();
}

Result SimRobotController::MoveLinear(const LinearMoveCommand&)
{
    if (!connected_)
        return Failure(ErrorCode::NotConnected, "SimRobotController: not connected");

    // 관절각→TCP FK는 역방향 TCP 목표→관절 IK나 직선 경로 실행을 제공하지 않는다.
    return Failure(
        ErrorCode::Unsupported,
        "SimRobotController: MoveLinear requires an IK solver and trajectory execution");
}

Result SimRobotController::Stop()
{
    if (!connected_)
        return Failure(ErrorCode::NotConnected, "SimRobotController: not connected");

    // 현재 q를 새 목표로 고정해 이후 Update가 남은 동작을 재개하지 않게 한다.
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
    // vector까지 값으로 복사하므로 호출자는 다음 Update와 무관한 snapshot을 받는다.
    return state_;
}

void SimRobotController::Update(double dtSeconds)
{
    // FixedControlLoop 같은 호출자가 정한 tick마다 q/dq만 갱신한다. FK와 Entity 반영은 별도 계층의 책임이다.
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

    // accelerationScale_는 공통 명령값 보존용이다. 이 backend는 속도 상한만 적용해 ramp를 만들지 않는다.
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
    // 생성 시 받은 사양을 복사하지 않고 참조한다. 따라서 사양 데이터를 가진 저장소는 이 Controller보다 오래 살아 있어야 한다.
    return *specification_;
}

} // grasplink::robotics::backends::simulation 네임스페이스
