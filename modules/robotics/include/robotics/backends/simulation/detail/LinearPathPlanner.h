#pragma once

#include "robotics/core/ControlTypes.h"
#include "robotics/backends/simulation/detail/SimulationMotionPolicy.h"
#include "robotics/kinematics/RobotInverseKinematics.h"
#include "robotics/models/RobotSpecification.h"

#include <cmath>
#include <functional>
#include <optional>
#include <vector>

namespace grasplink::robotics::backends::simulation::detail
{

inline kinematics::IkOptions PathIkOptions()
{
    kinematics::IkOptions options;
    // 경로의 각 표본을 직선 목표에 충분히 가깝게 맞추도록 일반 IK보다 엄격한 오차를 사용한다.
    options.positionToleranceMeters = 1e-7;
    options.orientationToleranceRadians = 1e-6;
    return options;
}

inline Result ValidateLinearPathCommand(const LinearPathMoveCommand& command)
{
    if (!command.targetPoses.empty() &&
        std::isfinite(command.maxLinearVelocityMetersPerSecond) && command.maxLinearVelocityMetersPerSecond > 0.0 &&
        std::isfinite(command.maxAngularVelocityRadiansPerSecond) && command.maxAngularVelocityRadiansPerSecond > 0.0 &&
        std::isfinite(command.maxLinearAccelerationMetersPerSecondSquared) && command.maxLinearAccelerationMetersPerSecondSquared > 0.0 &&
        std::isfinite(command.maxAngularAccelerationRadiansPerSecondSquared) && command.maxAngularAccelerationRadiansPerSecondSquared > 0.0)
        return Result::Success();
    return {ErrorCode::InvalidCommand, "SimRobotController: invalid linear path or motion limits"};
}

inline Result MapIkFailure(const kinematics::IkResult& result)
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
    return {code, result.message};
}

inline double RequiredTimeForVelocity(double displacement, double maximumVelocity)
{
    return std::abs(displacement) / maximumVelocity;
}

inline double VelocityRatio(double displacement, double availableSeconds, double maximumVelocity)
{
    return std::abs(displacement) / availableSeconds / maximumVelocity;
}

struct LinearPathPoint
{
    JointVector joints;
    CartesianPose tcpPose{};
    // 이전 표본부터 이 TCP·관절 변위를 각 속도 상한 안에서 완료하는 최소 시간 [s]다.
    double durationSeconds = 0.0;
};

struct LinearPathPlan
{
    std::vector<LinearPathPoint> points;
    double plannedLinearVelocity = 0.0;
    double plannedAngularVelocity = 0.0;
    bool hasMotion = false;
};

void AlignEquivalentJointAngles(
    JointVector& target,
    const JointVector& reference,
    const models::RobotSpecification& specification);

std::optional<kinematics::IkResult> SolveCollisionFreeIk(
    kinematics::DampedLeastSquaresIk& inverse,
    const models::RobotSpecification& specification,
    const std::function<JointStateInvalidity(const JointVector&)>& stateValidityChecker,
    const CartesianPose& target,
    const JointVector& start,
    const kinematics::IkOptions& options,
    const SimulationMotionPolicy& policy,
    JointStateInvalidity& invalidity,
    kinematics::IkResult& ikFailure);

/**
 * @brief 관절 자세가 유효한지 확인하고 첫 번째 실패 이유를 반환한다.
 * @details 관절 수, 유한한 각도, 관절 한계를 먼저 확인한다. 이 검사를 통과한 경우에만 등록된 충돌 검사기를 호출한다.
 */
JointStateInvalidity ValidateJointState(
    const models::RobotSpecification& specification,
    const JointVector& joints,
    const std::function<JointStateInvalidity(const JointVector&)>& stateValidityChecker);

/**
 * @brief 관절 자세의 실패 이유를 Controller가 반환하는 오류 결과로 바꾼다.
 * @details 환경 충돌, 로봇 자체 충돌, 붙어 있는 물체의 충돌을 서로 다른 ErrorCode로 유지한다.
 */
Result MapJointStateInvalidity(JointStateInvalidity invalidity);
// 경로 샘플 수가 제한을 넘으면 끝점 연관성과 도달 가능성을 검사해 오류를 구분한다.
Result BuildLinearPath(
    const LinearPathMoveCommand& command,
    const models::RobotSpecification& specification,
    const JointVector& startJoints,
    const CartesianPose& startTcp,
    kinematics::DampedLeastSquaresIk& inverse,
    const std::function<JointStateInvalidity(const JointVector&)>& stateValidityChecker,
    LinearPathPlan& plan,
    const SimulationMotionPolicy& policy = kDefaultSimulationMotionPolicy);

} // namespace grasplink::robotics::backends::simulation::detail
