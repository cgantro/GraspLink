#include "robotics/kinematics/GripperKinematics.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>

namespace grasplink::robotics::kinematics
{
namespace
{
bool IsFinite(const models::QuaternionWxyz& value)
{
    return std::isfinite(value.w) && std::isfinite(value.x) &&
        std::isfinite(value.y) && std::isfinite(value.z);
}

double AxisLength(const models::Axis3& axis)
{
    return std::hypot(axis.x, axis.y, axis.z);
}

models::QuaternionWxyz AxisRotation(const models::Axis3& axis, double angle)
{
    const double length = AxisLength(axis);
    const double halfAngle = angle * 0.5;
    const double sine = std::sin(halfAngle);
    return {
        std::cos(halfAngle),
        (axis.x / length) * sine,
        (axis.y / length) * sine,
        (axis.z / length) * sine};
}
}

GripperKinematics::GripperKinematics(const models::GripperSpecification& specification)
    : specification_(specification)
{
    if (specification_.joints == nullptr || specification_.jointCount == 0)
        throw std::invalid_argument("GripperKinematics: empty joint specification");
    if (!std::isfinite(specification_.nominalMasterClosedRadians) ||
        specification_.nominalMasterClosedRadians <= 0.0)
        throw std::invalid_argument("GripperKinematics: invalid nominal closed angle");

    std::unordered_set<std::string> names;
    names.reserve(specification_.jointCount);
    for (std::size_t i = 0; i < specification_.jointCount; ++i)
    {
        const auto& joint = specification_.joints[i];
        if (joint.name.empty() || !names.emplace(joint.name).second)
            throw std::invalid_argument("GripperKinematics: empty or duplicate joint name");

        const double axisLength = AxisLength(joint.axis);
        if (!std::isfinite(joint.axis.x) || !std::isfinite(joint.axis.y) ||
            !std::isfinite(joint.axis.z) || !std::isfinite(axisLength) || axisLength <= 0.0)
            throw std::invalid_argument("GripperKinematics: joint axis must be finite and nonzero");
        if (!std::isfinite(joint.masterMultiplier))
            throw std::invalid_argument("GripperKinematics: non-finite master multiplier");
        if (!std::isfinite(joint.minPositionRadians) || !std::isfinite(joint.maxPositionRadians) ||
            joint.minPositionRadians > joint.maxPositionRadians)
            throw std::invalid_argument("GripperKinematics: invalid joint limits");

        // closureFraction은 0에서 1까지 움직이므로 선형 각도의 양 끝을 제한 안에 둔다.
        const double closedAngle = joint.masterMultiplier * specification_.nominalMasterClosedRadians;
        if (!std::isfinite(closedAngle) || joint.minPositionRadians > 0.0 ||
            joint.maxPositionRadians < 0.0 || closedAngle < joint.minPositionRadians ||
            closedAngle > joint.maxPositionRadians)
            throw std::invalid_argument("GripperKinematics: joint limits exclude the nominal open-to-closed range");
    }

    state_.jointAnglesRadians.resize(specification_.jointCount);
    state_.jointLocalRotations.resize(specification_.jointCount);
    pending_ = state_;
}

const GripperKinematicState& GripperKinematics::Update(const GripperState& state)
{
    // inactive 여부는 위치 계산을 막지 않는다. 현재 pose의 유효성은 두 valid 표식과 fraction 값으로 판정한다.
    if (!state.valid || !state.closureFractionValid || !std::isfinite(state.closureFraction) ||
        state.closureFraction < 0.0 || state.closureFraction > 1.0)
        throw std::invalid_argument("GripperKinematics: invalid closure state");

    GripperKinematicState& next = pending_;
    next.masterAngleRadians = specification_.nominalMasterClosedRadians * state.closureFraction;

    for (std::size_t i = 0; i < specification_.jointCount; ++i)
    {
        const auto& joint = specification_.joints[i];
        const double angle = joint.masterMultiplier * next.masterAngleRadians;
        const models::QuaternionWxyz rotation = AxisRotation(joint.axis, angle);
        if (!std::isfinite(angle) || !IsFinite(rotation))
            throw std::invalid_argument("GripperKinematics: calculated non-finite joint pose");
        next.jointAnglesRadians[i] = angle;
        next.jointLocalRotations[i] = rotation;
    }

    // 모든 관절 결과를 먼저 계산해 검증한다. 중간 관절 오류가 직전 정상 상태 일부를 덮지 않게 한다.
    std::swap(state_, pending_);
    return state_;
}

} // namespace grasplink::robotics::kinematics
