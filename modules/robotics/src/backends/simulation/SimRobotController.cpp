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
 * @brief "목표각에 도달했다"고 판단할 때 허용하는 아주 작은 각도 오차 [rad].
 *
 * @details
 * double 계산에서는 수학적으로 같은 값도 bit 단위로 완전히 같지 않을 수 있다.
 * 그래서 차이가 1e-8 rad 이하이면 더 움직이지 않고 목표값에 정확히 맞춘 것으로 처리한다.
 */
constexpr double kPositionEpsilon = 1e-8;

/**
 * @brief 실패 Result를 짧게 만드는 내부 helper 함수.
 * @param code 실패 종류.
 * @param message 사람이 읽을 수 있는 실패 이유.
 * @return `Result{code, message}`.
 */
Result Failure(ErrorCode code, std::string message)
{
    return {code, std::move(message)};
}

/**
 * @brief 속도/가속도 scale이 현재 계약 `(0,1]` 범위의 정상 실수인지 확인한다.
 *
 * @details
 * scale은 물리 단위가 없는 배율이다.
 * 1.0은 모델 제한의 100%, 0.5는 50%를 뜻한다.
 */
bool IsScaleValid(double value)
{
    return std::isfinite(value) && value > 0.0 && value <= 1.0;
}
} // namespace

SimRobotController::SimRobotController(const models::RobotSpecification& specification)
    : specification_(&specification)
{
    // 관절 정의가 하나도 없는 모델은 Simulation Controller가 의미 있게 동작할 수 없다.
    if (specification_->joints == nullptr || specification_->jointCount == 0)
        throw std::invalid_argument("SimRobotController: empty robot specification");
}

Result SimRobotController::Connect()
{
    /*
     * q={0,...,0}, dq={0,...,0}의 논리적 simulation zero에서 시작한다.
     * q는 관절각, dq는 관절 각속도를 뜻하는 로보틱스의 흔한 표기다.
     *
     * 이 0은 실제 HCR encoder의 공장 calibration/home zero를 의미하지 않는다.
     * 실제 장비 offset은 별도의 Hardware/Calibration 계층에서 다뤄야 한다.
     */
    state_ = {};
    state_.jointPositionRadians.assign(specification_->jointCount, 0.0);
    state_.jointVelocityRadiansPerSecond.assign(specification_->jointCount, 0.0);
    state_.mode = RobotMode::Idle;
    state_.valid = true;
    state_.tcpPoseValid = false;

    // 처음에는 현재 자세가 곧 목표자세이므로 로봇이 움직이지 않는다.
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

    // 연결이 끊긴 상태에서 이전 속도값이 남아 "아직 움직인다"고 오해하지 않도록 0으로 만든다.
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

    /*
     * RobotSpecification의 jointCount/order가 모든 JointVector의 기준이다.
     * HCR-12A라면 정확히 6개이고 순서는 J1,J2,J3,J4,J5,J6다.
     */
    if (command.targetPositionRadians.size() != specification_->jointCount)
        return Failure(ErrorCode::InvalidCommand, "SimRobotController: joint count mismatch");

    if (!IsScaleValid(command.velocityScale) || !IsScaleValid(command.accelerationScale))
        return Failure(ErrorCode::InvalidCommand, "SimRobotController: scale must be in (0, 1]");

    /*
     * Joint limit은 "이 관절이 허용되는 최소/최대 각도 범위"다.
     * 범위를 넘은 command를 임의로 경계값에 붙여(clamp) 실행하지 않고 명령 자체를 거절한다.
     * 이렇게 해야 호출자가 잘못된 목표를 보냈다는 사실을 숨기지 않는다.
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

    // 검증이 끝난 뒤에만 새 target과 scale을 저장한다.
    targetPositionRadians_ = command.targetPositionRadians;
    velocityScale_ = command.velocityScale;
    accelerationScale_ = command.accelerationScale;

    // 현재 q와 target q가 하나라도 다르면 Moving, 전부 이미 같으면 Idle이다.
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

    /*
     * Cartesian target은 TCP의 위치/방향으로 주어진다.
     * 이를 실제 J1..Jn 목표각으로 바꾸려면 IK가 필요하고,
     * 시간에 따른 직선 경로를 만들려면 trajectory 계층도 필요하다.
     * 아직 둘 다 연결되지 않았으므로 의도적으로 Unsupported를 반환한다.
     */
    return Failure(
        ErrorCode::Unsupported,
        "SimRobotController: MoveLinear requires the FK/IK layer and is not wired yet");
}

Result SimRobotController::Stop()
{
    if (!connected_)
        return Failure(ErrorCode::NotConnected, "SimRobotController: not connected");

    /*
     * 새 target을 현재 q와 같게 만들면 다음 Update에서 더 이동할 이유가 없어진다.
     * 이것은 software stop이며 실제 산업용 로봇의 안전회로 E-Stop을 의미하지 않는다.
     */
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
    // snapshot을 값으로 복사해 외부 코드가 Controller 내부 state를 직접 수정하지 못하게 한다.
    return state_;
}

void SimRobotController::Update(double dtSeconds)
{
    // 움직이는 상태이고 dt가 정상적인 양수일 때만 simulation step을 진행한다.
    if (!connected_ || state_.mode != RobotMode::Moving ||
        !std::isfinite(dtSeconds) || dtSeconds <= 0.0)
    {
        return;
    }

    bool allReached = true;

    for (std::size_t i = 0; i < specification_->jointCount; ++i)
    {
        const auto& joint = specification_->joints[i];

        /*
         * delta는 목표각까지 남은 "부호가 있는 각도 차이" [rad].
         * 양수면 +axis 방향으로, 음수면 반대 방향으로 더 움직여야 한다.
         */
        const double delta = targetPositionRadians_[i] - state_.jointPositionRadians[i];

        if (std::abs(delta) <= kPositionEpsilon)
        {
            state_.jointPositionRadians[i] = targetPositionRadians_[i];
            state_.jointVelocityRadiansPerSecond[i] = 0.0;
            continue;
        }

        /*
         * 이번 한 step에서 움직일 수 있는 최대 각도 계산:
         *
         * maxStep[rad]
         * = 모델 최대각속도[rad/s]
         * * 사용자가 요청한 속도비율[-]
         * * 이번 step 시간[s]
         *
         * 예) 2 rad/s * 0.5 * 0.01 s = 0.01 rad
         */
        const double maxStep = joint.maxVelocityRadiansPerSecond * velocityScale_ * dtSeconds;
        if (maxStep <= 0.0)
        {
            state_.jointVelocityRadiansPerSecond[i] = 0.0;
            allReached = false;
            continue;
        }

        /*
         * std::clamp로 delta를 [-maxStep,+maxStep] 범위에 제한한다.
         * 그래서 목표를 향해 최대속도 이하로 움직이면서 목표각을 지나쳐 overshoot하지 않는다.
         */
        const double step = std::clamp(delta, -maxStep, maxStep);
        state_.jointPositionRadians[i] += step;

        // 실제 이번 step 이동각 / 시간으로 현재 feedback 각속도 [rad/s]를 계산한다.
        state_.jointVelocityRadiansPerSecond[i] = step / dtSeconds;

        if (std::abs(targetPositionRadians_[i] - state_.jointPositionRadians[i]) > kPositionEpsilon)
        {
            allReached = false;
        }
        else
        {
            // epsilon 안에 들어오면 오차를 남기지 않고 target에 정확히 맞춘다.
            state_.jointPositionRadians[i] = targetPositionRadians_[i];
        }
    }

    /*
     * accelerationScale_은 API 계약에는 존재하지만 아직 실제 제한에 쓰지 않는다.
     * HCR specification에 검증된 max acceleration을 넣기 전 임의 숫자로 가속도를 제한하면
     * "제조사 사양을 구현했다"는 잘못된 인상을 줄 수 있기 때문이다.
     */
    (void)accelerationScale_;

    if (allReached)
    {
        // 모든 관절이 목표각에 도달하면 속도 0, 상태 Idle로 전환한다.
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
