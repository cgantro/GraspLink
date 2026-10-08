#pragma once

#include "robotics/core/ControlTypes.h"
#include "robotics/planning/PlanningPolicy.h"
#include "robotics/planning/StateValidity.h"
#include "robotics/kinematics/RobotInverseKinematics.h"
#include "robotics/models/RobotSpecification.h"

#include <cmath>
#include <functional>
#include <optional>
#include <vector>

namespace grasplink::robotics::planning
{

/** @brief 직선 경로의 각 표본을 더 엄격한 위치·방향 오차로 맞추는 IK 설정을 만든다. */
inline kinematics::IkOptions PathIkOptions()
{
    kinematics::IkOptions options;
    // 경로의 각 표본을 직선 목표에 충분히 가깝게 맞추도록 일반 IK보다 엄격한 오차를 사용한다.
    options.positionToleranceMeters = 1e-7;
    options.orientationToleranceRadians = 1e-6;
    return options;
}

/** @brief TCP 경로가 비어 있지 않고 속도·가속도 상한이 모두 유효한지 확인한다. */
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

/** @brief IK 실패 상태를 공통 Controller 결과 코드로 바꾼다. */
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

/** @brief 변위를 지정한 최대 속도 이하로 이동하는 데 필요한 최소 시간 [s]를 계산한다. */
inline double RequiredTimeForVelocity(double displacement, double maximumVelocity)
{
    return std::abs(displacement) / maximumVelocity;
}

/** @brief 주어진 시간 동안 이동할 때 최대 속도에 대한 요구 속도 비율을 계산한다. */
inline double VelocityRatio(double displacement, double availableSeconds, double maximumVelocity)
{
    return std::abs(displacement) / availableSeconds / maximumVelocity;
}

/** @brief 계획된 관절 자세와 TCP 자세, 이전 표본에서 이동하는 최소 시간을 보관한다. */
struct LinearPathPoint
{
    JointVector joints;
    CartesianPose tcpPose{};
    // 이전 표본부터 이 TCP·관절 변위를 각 속도 상한 안에서 완료하는 최소 시간 [s]다.
    double durationSeconds = 0.0;
};

/** @brief TCP 경로를 실행할 관절 표본과 경로 전체의 계획 속도를 보관한다. */
struct LinearPathPlan
{
    std::vector<LinearPathPoint> points;
    double plannedLinearVelocity = 0.0;
    double plannedAngularVelocity = 0.0;
    bool hasMotion = false;
};

/** @brief 허용 관절 범위 안에서 기준 자세에 가장 가까운 2π 등가 목표각을 선택한다. */
void AlignEquivalentJointAngles(
    JointVector& target,
    const JointVector& reference,
    const models::RobotSpecification& specification);

/** @brief TCP 목표 IK 해 가운데 시작 자세부터의 관절 경로가 유효한 해를 선택한다. */
std::optional<kinematics::IkResult> SolveCollisionFreeIk(
    kinematics::DampedLeastSquaresIk& inverse,
    const models::RobotSpecification& specification,
    const StateValidityChecker& stateValidityChecker,
    const CartesianPose& target,
    const JointVector& start,
    const kinematics::IkOptions& options,
    const PlanningPolicy& policy,
    JointStateInvalidity& invalidity,
    kinematics::IkResult& ikFailure);

/**
 * @brief 시작 관절각에서 목표 관절각까지 충돌 없이 이동할 수 있는지 확인한다.
 * @details 허용된 관절 변화 간격으로 중간 자세를 검사하므로, 두 끝점이 모두 안전하더라도 그 사이 경로가 충돌하면 실패한다.
 */
[[nodiscard]] JointStateInvalidity ValidateJointPath(
    const JointVector& start,
    const JointVector& end,
    const models::RobotSpecification& specification,
    const StateValidityChecker& stateValidityChecker,
    const PlanningPolicy& policy = kDefaultPlanningPolicy);

/**
 * @brief TCP 직선 경로를 IK 표본과 관절 경로 검증으로 바꾼다.
 * @details 경로가 유효한지 전부 확인한 뒤에만 결과 계획을 갱신한다. 샘플 한도를 넘으면 끝점의 도달 가능성과 경로 구간 수 초과를 구분한다.
 */
Result BuildLinearPath(
    const LinearPathMoveCommand& command,
    const models::RobotSpecification& specification,
    const JointVector& startJoints,
    const CartesianPose& startTcp,
    kinematics::DampedLeastSquaresIk& inverse,
    const StateValidityChecker& stateValidityChecker,
    LinearPathPlan& plan,
    const PlanningPolicy& policy = kDefaultPlanningPolicy);

} // namespace grasplink::robotics::planning
