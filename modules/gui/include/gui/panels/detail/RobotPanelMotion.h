#pragma once

#include "robotics/core/ControlTypes.h"
#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/models/RobotSpecification.h"

#include <glm/gtc/quaternion.hpp>

#include <array>
#include <algorithm>
#include <cmath>
#include <optional>
#include <string_view>

namespace grasplink::gui::detail
{

struct WristOnlyTarget
{
    robotics::JointVector jointPositionRadians;
    double rotationRadians = 0.0;
};

/**
 * @brief 지정한 TCP 방향을 J6만 돌려 만들 수 있을 때 그 관절 목표를 계산한다.
 * @details 현재 TCP와 파지 물체 중심이 J6 회전축에서 2 mm보다 더 벗어나 있으면 회전 중 크게 쓸릴 수 있으므로 계산을 거부한다.
 * 관절 목표는 J1부터 J5까지 현재값을 그대로 복사하고, J6의 2π 등가값 중 실제 허용 범위에 들어오는 값을 고른다.
 */
inline std::optional<WristOnlyTarget> PlanWristOnlyTarget(
    const robotics::models::RobotSpecification& specification,
    const robotics::RobotState& state,
    const std::array<double, 4>& targetOrientationXyzw,
    const std::optional<std::array<double, 3>>& boxOffsetInTcpMeters = std::nullopt)
{
    if (!state.valid || !state.tcpPoseValid || specification.joints == nullptr ||
        specification.jointCount == 0 || state.jointPositionRadians.size() != specification.jointCount ||
        std::string_view(specification.joints[specification.jointCount - 1].name) != "J6")
        return std::nullopt;

    const glm::dquat currentInput{
        state.tcpPose.orientationXyzw[3], state.tcpPose.orientationXyzw[0],
        state.tcpPose.orientationXyzw[1], state.tcpPose.orientationXyzw[2]};
    const glm::dquat targetInput{
        targetOrientationXyzw[3], targetOrientationXyzw[0],
        targetOrientationXyzw[1], targetOrientationXyzw[2]};
    if (!std::isfinite(glm::length(currentInput)) || !std::isfinite(glm::length(targetInput)) ||
        glm::length(currentInput) <= 1e-12 || glm::length(targetInput) <= 1e-12)
        return std::nullopt;
    const glm::dquat currentOrientation = glm::normalize(currentInput);
    const glm::dquat targetOrientation = glm::normalize(targetInput);

    robotics::kinematics::RobotKinematics kinematics(specification);
    const auto& pose = kinematics.Update(state.jointPositionRadians);
    const glm::dvec3 axis = glm::normalize(glm::dvec3{
        pose.jointAxesInBaseFrame.back().x,
        pose.jointAxesInBaseFrame.back().y,
        pose.jointAxesInBaseFrame.back().z});
    const glm::dvec3 pivot{
        pose.linkPosesInBaseFrame.back().positionMeters.x,
        pose.linkPosesInBaseFrame.back().positionMeters.y,
        pose.linkPosesInBaseFrame.back().positionMeters.z};
    const glm::dvec3 tcpPosition{
        state.tcpPose.positionMeters[0], state.tcpPose.positionMeters[1], state.tcpPose.positionMeters[2]};
    constexpr double maximumRadialOffsetMeters = 0.002;
    const auto radialOffset = [&](const glm::dvec3& point)
    {
        const glm::dvec3 fromPivot = point - pivot;
        return fromPivot - axis * glm::dot(fromPivot, axis);
    };
    if (glm::length(radialOffset(tcpPosition)) > maximumRadialOffsetMeters)
        return std::nullopt;

    if (boxOffsetInTcpMeters)
    {
        const glm::dvec3 localOffset{(*boxOffsetInTcpMeters)[0], (*boxOffsetInTcpMeters)[1],
            (*boxOffsetInTcpMeters)[2]};
        const glm::dvec3 boxCenter = tcpPosition + currentOrientation * localOffset;
        if (glm::length(radialOffset(boxCenter)) > maximumRadialOffsetMeters)
            return std::nullopt;
    }

    glm::dquat difference = glm::normalize(targetOrientation * glm::inverse(currentOrientation));
    if (difference.w < 0.0)
        difference = -difference;
    const glm::dvec3 vectorPart{difference.x, difference.y, difference.z};
    const double vectorLength = glm::length(vectorPart);
    double signedRotation = 0.0;
    if (vectorLength > 1e-10)
    {
        const glm::dvec3 rotationAxis = vectorPart / vectorLength;
        const double axisAlignment = glm::dot(rotationAxis, axis);
        if (std::abs(axisAlignment) < 1.0 - 1e-6)
            return std::nullopt;
        signedRotation = 2.0 * std::atan2(vectorLength, difference.w) * (axisAlignment < 0.0 ? -1.0 : 1.0);
    }

    const auto& wrist = specification.joints[specification.jointCount - 1];
    constexpr double fullTurn = 6.28318530717958647692;
    double targetWrist = state.jointPositionRadians.back() + signedRotation;
    if (targetWrist < wrist.minPositionRadians || targetWrist > wrist.maxPositionRadians)
    {
        const double minimumTurns = std::ceil((wrist.minPositionRadians - targetWrist) / fullTurn);
        const double maximumTurns = std::floor((wrist.maxPositionRadians - targetWrist) / fullTurn);
        if (minimumTurns > maximumTurns)
            return std::nullopt;
        const double nearestTurns = std::round((state.jointPositionRadians.back() - targetWrist) / fullTurn);
        targetWrist += std::clamp(nearestTurns, minimumTurns, maximumTurns) * fullTurn;
    }

    WristOnlyTarget result;
    result.jointPositionRadians = state.jointPositionRadians;
    result.jointPositionRadians.back() = targetWrist;
    result.rotationRadians = signedRotation;
    return result;
}

} // namespace grasplink::gui::detail
