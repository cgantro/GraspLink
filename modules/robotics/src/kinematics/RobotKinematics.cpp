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
    // FK(정방향 기구학)는 관절각에서 각 링크 자세를 계산한다. 계산에 앞서 모델에 관절 pivot(회전 중심), 축, 선택적 ToolFrame이 유효하게 정의되어 있는지 확인한다.
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
    state_.jointAxesInBaseFrame.resize(specification_.jointCount);
}

const RobotKinematicState& RobotKinematics::Update(const RobotState& state)
{
    // 결과 위치와 회전은 Robot base 기준이다. Scene에서 robotRoot에 적용한 배치 위치와 회전은 아직 포함하지 않는다.
    if (!state.valid)
        throw std::invalid_argument("RobotKinematics: invalid RobotState");
    return Update(state.jointPositionRadians);
}

const RobotKinematicState& RobotKinematics::Update(const JointVector& jointPositionRadians)
{
    if (jointPositionRadians.size() != specification_.jointCount)
        throw std::invalid_argument("RobotKinematics: joint count mismatch");
    for (double angle : jointPositionRadians)
        if (!std::isfinite(angle))
            throw std::invalid_argument("RobotKinematics: non-finite joint angle");

    QuaternionWxyz parentRotation{};
    Vec3 parentPosition{};
    Vec3 previousBindPivot{};

    for (std::size_t i = 0; i < specification_.jointCount; ++i)
    {
        const models::JointSpecification& joint = specification_.joints[i];
        const double angle = jointPositionRadians[i];

        // 모델 초기 상태(bind pose)에서 이웃한 관절 중심 사이의 차이를 구한다. 부모 관절이 회전하면 이 간격도 함께 회전하므로, 누적된 부모 회전을 적용해 현재 관절 중심을 계산한다.
        const Vec3 bindOffset = i == 0
            ? joint.bindPivotMeters
            : Subtract(joint.bindPivotMeters, previousBindPivot);
        const Vec3 jointPosition = Add(parentPosition, Rotate(parentRotation, bindOffset));
        // 관절 Local 축을 부모 누적 회전으로 Robot base 축으로 바꾼다. 자체 회전은 자기 축 방향을 바꾸지 않는다.
        const double axisLength = Length(joint.axis);
        state_.jointAxesInBaseFrame[i] = Rotate(parentRotation,
            {joint.axis.x / axisLength, joint.axis.y / axisLength, joint.axis.z / axisLength});
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
        // ToolFrame은 모델에 고정된 공구 장착 기준점이며 공구 끝 TCP와 다를 수 있다.
        // 여기서 계산한 모델 자세는 Controller가 실제로 보고한 tcpPose feedback이 아니다.
        const auto& tool = specification_.toolFrameInLastJoint;
        state_.toolFrameInBaseFrame = {
            Add(parentPosition, Rotate(parentRotation, tool.positionMeters)),
            Multiply(parentRotation, Normalize(tool.rotation))};
    }
    return state_;
}

}
