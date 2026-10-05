#include "robotics/kinematics/RobotKinematics.h"

#include <cmath>
#include <stdexcept>

namespace grasplink::robotics::kinematics
{
namespace
{
using models::Axis3;
using models::Pose3;
using models::QuaternionWxyz;
using models::Vec3;

double Length(const Vec3& value)
{
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

Vec3 Add(const Vec3& left, const Vec3& right)
{
    return {left.x + right.x, left.y + right.y, left.z + right.z};
}

Vec3 Subtract(const Vec3& left, const Vec3& right)
{
    return {left.x - right.x, left.y - right.y, left.z - right.z};
}

QuaternionWxyz Normalize(const QuaternionWxyz& value)
{
    const double length = std::sqrt(
        value.w * value.w + value.x * value.x + value.y * value.y + value.z * value.z);
    if (!std::isfinite(length) || length <= 1.0e-12)
        throw std::invalid_argument("RobotKinematics: invalid quaternion");
    return {value.w / length, value.x / length, value.y / length, value.z / length};
}

QuaternionWxyz Multiply(const QuaternionWxyz& left, const QuaternionWxyz& right)
{
    return Normalize({
        left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z,
        left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y,
        left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x,
        left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w});
}

QuaternionWxyz MultiplyRaw(const QuaternionWxyz& left, const QuaternionWxyz& right)
{
    return {
        left.w * right.w - left.x * right.x - left.y * right.y - left.z * right.z,
        left.w * right.x + left.x * right.w + left.y * right.z - left.z * right.y,
        left.w * right.y - left.x * right.z + left.y * right.w + left.z * right.x,
        left.w * right.z + left.x * right.y - left.y * right.x + left.z * right.w};
}

Vec3 Rotate(const QuaternionWxyz& rotation, const Vec3& value)
{
    const QuaternionWxyz vector{0.0, value.x, value.y, value.z};
    const QuaternionWxyz inverse{rotation.w, -rotation.x, -rotation.y, -rotation.z};
    const QuaternionWxyz result = MultiplyRaw(MultiplyRaw(rotation, vector), inverse);
    return {result.x, result.y, result.z};
}

QuaternionWxyz AxisRotation(const Axis3& axis, double angle)
{
    const double length = Length(axis);
    if (!std::isfinite(length) || length <= 1.0e-12)
        throw std::invalid_argument("RobotKinematics: invalid joint axis");

    const double halfAngle = angle * 0.5;
    const double scale = std::sin(halfAngle) / length;
    return Normalize({std::cos(halfAngle), axis.x * scale, axis.y * scale, axis.z * scale});
}
}

RobotKinematics::RobotKinematics(const models::RobotSpecification& specification)
    : specification_(specification)
{
    // 참조 모델의 pivot, 축, 선택적 ToolFrame이 FK 계산에 쓸 수 있는지 먼저 확인한다.
    if (specification_.joints == nullptr || specification_.jointCount == 0)
        throw std::invalid_argument("RobotKinematics: empty robot specification");

    for (std::size_t i = 0; i < specification_.jointCount; ++i)
    {
        const auto& joint = specification_.joints[i];
        if (!std::isfinite(joint.bindPivotMeters.x) || !std::isfinite(joint.bindPivotMeters.y) ||
            !std::isfinite(joint.bindPivotMeters.z))
            throw std::invalid_argument("RobotKinematics: non-finite bind pivot");
        AxisRotation(joint.axis, 0.0);
    }
    if (specification_.hasToolFrame)
    {
        const auto& tool = specification_.toolFrameInLastJoint;
        if (!std::isfinite(tool.positionMeters.x) || !std::isfinite(tool.positionMeters.y) ||
            !std::isfinite(tool.positionMeters.z))
            throw std::invalid_argument("RobotKinematics: non-finite tool offset");
        Normalize(tool.rotation);
    }

    state_.jointLocalRotations.resize(specification_.jointCount);
    state_.linkPosesInBaseFrame.resize(specification_.jointCount);
}

const RobotKinematicState& RobotKinematics::Update(const RobotState& state)
{
    // 좌표: Robot base 기준 결과. Scene에서 배치한 root의 위치와 회전은 포함하지 않음.
    if (!state.valid)
        throw std::invalid_argument("RobotKinematics: invalid RobotState");
    if (state.jointPositionRadians.size() != specification_.jointCount)
        throw std::invalid_argument("RobotKinematics: joint count mismatch");

    QuaternionWxyz parentRotation{};
    Vec3 parentPosition{};
    Vec3 previousBindPivot{};

    for (std::size_t i = 0; i < specification_.jointCount; ++i)
    {
        const models::JointSpecification& joint = specification_.joints[i];
        const double angle = state.jointPositionRadians[i];
        if (!std::isfinite(angle))
            throw std::invalid_argument("RobotKinematics: non-finite joint angle");

        // 계산: 연속한 base-frame bind pivot 차이를 부모 누적 회전으로 옮겨 현재 관절 중심을 구한다.
        const Vec3 bindOffset = i == 0
            ? joint.bindPivotMeters
            : Subtract(joint.bindPivotMeters, previousBindPivot);
        const Vec3 jointPosition = Add(parentPosition, Rotate(parentRotation, bindOffset));
        // 회전: axis-angle은 관절 Local 회전이며 parent * local로 누적한다. Local 점에는 관절 회전 후 부모 회전이 적용된다.
        const QuaternionWxyz localRotation = AxisRotation(joint.axis, angle);
        const QuaternionWxyz worldRotation = Multiply(parentRotation, localRotation);

        state_.jointLocalRotations[i] = localRotation;
        state_.linkPosesInBaseFrame[i] = {jointPosition, worldRotation};
        parentPosition = jointPosition;
        parentRotation = worldRotation;
        previousBindPivot = joint.bindPivotMeters;
    }

    state_.toolFrameValid = specification_.hasToolFrame;
    if (state_.toolFrameValid)
    {
        // ToolFrame의 고정 강체 변환을 마지막 joint에 합성한다. Controller tcpPose feedback과 독립된 예측값이다.
        const auto& tool = specification_.toolFrameInLastJoint;
        state_.toolFrameInBaseFrame = {
            Add(parentPosition, Rotate(parentRotation, tool.positionMeters)),
            Multiply(parentRotation, Normalize(tool.rotation))};
    }
    return state_;
}

}
