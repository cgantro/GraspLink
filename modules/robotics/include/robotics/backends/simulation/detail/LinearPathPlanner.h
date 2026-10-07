#pragma once

#include "robotics/core/ControlTypes.h"
#include "robotics/kinematics/RobotInverseKinematics.h"
#include "robotics/models/RobotSpecification.h"

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

using CollisionAwareIkSolver = std::function<std::optional<kinematics::IkResult>(
    const CartesianPose&, const JointVector&, const kinematics::IkOptions&, bool&, kinematics::IkResult&)>;
// 표본 한도 초과 시 끝점 자체가 도달 불가능한지와 경로 해상도만 부족한지를 구분한다.
using EndpointReachabilitySolver = std::function<kinematics::IkResult(const CartesianPose&, const JointVector&)>;

class LinearPathPlanner
{
public:
    static Result Build(
        const LinearPathMoveCommand& command,
        const models::RobotSpecification& specification,
        const JointVector& startJoints,
        const CartesianPose& startTcp,
        const CollisionAwareIkSolver& solveCollisionFreeIk,
        const EndpointReachabilitySolver& solveEndpointReachability,
        LinearPathPlan& plan);
};

} // namespace grasplink::robotics::backends::simulation::detail
