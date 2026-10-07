#include "robotics/planning/WristAlignmentPlanner.h"

#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/kinematics/detail/PoseMath.h"

#include <algorithm>
#include <cmath>
#include <string_view>

namespace grasplink::robotics::planning
{
namespace
{
using kinematics::detail::Add;
using kinematics::detail::Length;
using kinematics::detail::Multiply;
using kinematics::detail::Normalize;
using kinematics::detail::Rotate;
using kinematics::detail::Scale;
using kinematics::detail::Subtract;
using models::QuaternionWxyz;
using models::Vec3;

double Dot(const Vec3& a, const Vec3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

bool HasUsableMagnitude(const QuaternionWxyz& value)
{
    const double length = std::sqrt(value.w * value.w + value.x * value.x + value.y * value.y + value.z * value.z);
    return std::isfinite(length) && length > 1.0e-12;
}

Vec3 RadialOffset(const Vec3& point, const Vec3& pivot, const Vec3& axis)
{
    const Vec3 fromPivot = Subtract(point, pivot);
    return Subtract(fromPivot, Scale(axis, Dot(fromPivot, axis)));
}
}

std::optional<WristAlignmentTarget> PlanWristOnlyTarget(
    const models::RobotSpecification& specification,
    const RobotState& state,
    const std::array<double, 4>& targetOrientationXyzw,
    const std::optional<std::array<double, 3>>& boxOffsetInTcpMeters)
{
    if (!state.valid || !state.tcpPoseValid || specification.joints == nullptr ||
        specification.jointCount == 0 || state.jointPositionRadians.size() != specification.jointCount ||
        std::string_view(specification.joints[specification.jointCount - 1].name) != "J6")
        return std::nullopt;

    const QuaternionWxyz currentInput{
        state.tcpPose.orientationXyzw[3], state.tcpPose.orientationXyzw[0],
        state.tcpPose.orientationXyzw[1], state.tcpPose.orientationXyzw[2]};
    const QuaternionWxyz targetInput{
        targetOrientationXyzw[3], targetOrientationXyzw[0],
        targetOrientationXyzw[1], targetOrientationXyzw[2]};
    if (!HasUsableMagnitude(currentInput) || !HasUsableMagnitude(targetInput))
        return std::nullopt;
    const QuaternionWxyz currentOrientation = Normalize(currentInput);
    const QuaternionWxyz targetOrientation = Normalize(targetInput);

    kinematics::RobotKinematics robotKinematics(specification);
    const auto& pose = robotKinematics.Update(state.jointPositionRadians);
    const Vec3 rawAxis = pose.jointAxesInBaseFrame.back();
    const Vec3 axis = Scale(rawAxis, 1.0 / Length(rawAxis));
    const Vec3 pivot = pose.linkPosesInBaseFrame.back().positionMeters;
    const Vec3 tcpPosition{
        state.tcpPose.positionMeters[0], state.tcpPose.positionMeters[1], state.tcpPose.positionMeters[2]};
    constexpr double maximumRadialOffsetMeters = 0.002;
    constexpr double maximumAttachedBoxSweepMeters = 0.02;
    double attachedBoxRadialOffsetMeters = 0.0;
    if (Length(RadialOffset(tcpPosition, pivot, axis)) > maximumRadialOffsetMeters)
        return std::nullopt;

    if (boxOffsetInTcpMeters)
    {
        const Vec3 localOffset{
            (*boxOffsetInTcpMeters)[0], (*boxOffsetInTcpMeters)[1], (*boxOffsetInTcpMeters)[2]};
        const Vec3 boxCenter = Add(tcpPosition, Rotate(currentOrientation, localOffset));
        attachedBoxRadialOffsetMeters = Length(RadialOffset(boxCenter, pivot, axis));
    }

    QuaternionWxyz difference = Normalize(Multiply(targetOrientation,
        {currentOrientation.w, -currentOrientation.x, -currentOrientation.y, -currentOrientation.z}));
    if (difference.w < 0.0)
        difference = {-difference.w, -difference.x, -difference.y, -difference.z};
    const Vec3 vectorPart{difference.x, difference.y, difference.z};
    const double vectorLength = Length(vectorPart);
    double signedRotation = 0.0;
    if (vectorLength > 1.0e-10)
    {
        const Vec3 rotationAxis = Scale(vectorPart, 1.0 / vectorLength);
        const double axisAlignment = Dot(rotationAxis, axis);
        if (std::abs(axisAlignment) < 1.0 - 1.0e-6)
            return std::nullopt;
        signedRotation = 2.0 * std::atan2(vectorLength, difference.w) * (axisAlignment < 0.0 ? -1.0 : 1.0);
    }

    const double attachedBoxSweepMeters = 2.0 * attachedBoxRadialOffsetMeters *
        std::sin(std::abs(signedRotation) * 0.5);
    if (attachedBoxSweepMeters > maximumAttachedBoxSweepMeters)
        return std::nullopt;

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

    WristAlignmentTarget result;
    result.jointPositionRadians = state.jointPositionRadians;
    result.jointPositionRadians.back() = targetWrist;
    result.rotationRadians = signedRotation;
    result.attachedBoxSweepMeters = attachedBoxSweepMeters;
    return result;
}

} // namespace grasplink::robotics::planning
