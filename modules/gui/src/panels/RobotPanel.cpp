#include "gui/panels/RobotPanel.h"
#include "gui/panels/detail/RobotPanelMotion.h"

#include "robotics/backends/simulation/SimRobotController.h"

#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>
#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <string_view>
#include <utility>

namespace grasplink::gui
{
namespace
{
constexpr double kApproachHeightMeters = 0.25;
constexpr double kTransitBoxHeightMeters = 0.45;
constexpr double kBaseExclusionRadiusMeters = 0.42;
constexpr double kTransitAnnulusRadiusMeters = 0.56;
// 실제 Gripper convex hull은 TCP가 상자 중심에서 10 mm 위일 때 바닥과 겹쳤다. 20 mm에서 간격이 생겨 여유 5 mm를 더 둔다.
constexpr double kGraspClearanceMeters = 0.025;
// 작업공간 상한은 안전 여유값으로 두고, 각 경로 구간의 실제 속도는 IK가 만든 관절 변화량과 모델 관절 속도로 정한다.
constexpr double kLinearVelocityMetersPerSecond = 2.0;
constexpr double kAngularVelocityRadiansPerSecond = 8.0;
constexpr double kLinearAccelerationMetersPerSecondSquared = 30.0;
constexpr double kAngularAccelerationRadiansPerSecondSquared = 120.0;
constexpr double kJ6UnwindToleranceRadians = glm::radians(1.0);
constexpr double kBoxSideMeters = 0.04;
constexpr double kPlacementAreaSideMeters = kBoxSideMeters * 1.7320508;

glm::dquat TcpOrientation(const robotics::RobotState& state)
{
    return {state.tcpPose.orientationXyzw[3], state.tcpPose.orientationXyzw[0],
        state.tcpPose.orientationXyzw[1], state.tcpPose.orientationXyzw[2]};
}

glm::dquat Orientation(const std::array<double, 4>& xyzw)
{
    return glm::normalize(glm::dquat{xyzw[3], xyzw[0], xyzw[1], xyzw[2]});
}

std::array<double, 4> QuaternionXyzw(const glm::dquat& orientation)
{
    const auto normalized = glm::normalize(orientation);
    return {normalized.x, normalized.y, normalized.z, normalized.w};
}

double YawDegrees(const std::array<double, 4>& xyzw)
{
    const glm::dquat rotation = Orientation(xyzw);
    return glm::degrees(std::atan2(2.0 * (rotation.w * rotation.y + rotation.x * rotation.z),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)));
}

bool BoxCornersFitPlacement(const robotics::CartesianPose& boxPose,
    const robotics::CartesianPose& placementPose)
{
    const glm::dvec3 boxCenter{boxPose.positionMeters[0], boxPose.positionMeters[1], boxPose.positionMeters[2]};
    const glm::dvec3 targetCenter{placementPose.positionMeters[0], placementPose.positionMeters[1], placementPose.positionMeters[2]};
    const glm::dquat targetOrientation = Orientation(placementPose.orientationXyzw);
    const glm::dvec3 centerInTarget = glm::inverse(targetOrientation) * (boxCenter - targetCenter);
    const glm::dquat boxOrientationInTarget = glm::inverse(targetOrientation) * Orientation(boxPose.orientationXyzw);
    const double boxHalfSide = kBoxSideMeters * 0.5;
    const double targetHalfSide = kPlacementAreaSideMeters * 0.5;
    for (const double xSign : {-1.0, 1.0})
        for (const double zSign : {-1.0, 1.0})
        {
            const glm::dvec3 boxCorner = centerInTarget + boxOrientationInTarget *
                glm::dvec3{xSign * boxHalfSide, 0.0, zSign * boxHalfSide};
            if (std::abs(boxCorner.x) > targetHalfSide || std::abs(boxCorner.z) > targetHalfSide)
                return false;
        }
    return true;
}

bool ChordEntersBaseExclusion(const robotics::CartesianPose& start,
    const robotics::CartesianPose& end)
{
    const double dx = end.positionMeters[0] - start.positionMeters[0];
    const double dz = end.positionMeters[2] - start.positionMeters[2];
    const double lengthSquared = dx * dx + dz * dz;
    const double fraction = lengthSquared > 1e-12
        ? std::clamp(-(start.positionMeters[0] * dx + start.positionMeters[2] * dz) / lengthSquared, 0.0, 1.0)
        : 0.0;
    const double closestX = start.positionMeters[0] + fraction * dx;
    const double closestZ = start.positionMeters[2] + fraction * dz;
    return closestX * closestX + closestZ * closestZ <
        kBaseExclusionRadiusMeters * kBaseExclusionRadiusMeters;
}

std::size_t BuildTransitWaypoints(const robotics::CartesianPose& start,
    const robotics::CartesianPose& end, std::array<robotics::CartesianPose, 6>& waypoints)
{
    if (!ChordEntersBaseExclusion(start, end))
    {
        waypoints[0] = end;
        return 1;
    }

    const double startAngle = std::atan2(start.positionMeters[2], start.positionMeters[0]);
    const double endAngle = std::atan2(end.positionMeters[2], end.positionMeters[0]);
    double arcAngle = std::remainder(endAngle - startAngle, glm::two_pi<double>());
    const std::size_t arcSegments = std::max<std::size_t>(1,
        static_cast<std::size_t>(std::ceil(std::abs(arcAngle) / glm::quarter_pi<double>())));
    std::size_t count = 0;
    const auto setAnnulusPoint = [&](double angle)
    {
        robotics::CartesianPose point = start;
        point.positionMeters[0] = std::cos(angle) * kTransitAnnulusRadiusMeters;
        point.positionMeters[2] = std::sin(angle) * kTransitAnnulusRadiusMeters;
        waypoints[count++] = point;
    };

    setAnnulusPoint(startAngle);
    for (std::size_t segment = 1; segment <= arcSegments; ++segment)
        setAnnulusPoint(startAngle + arcAngle * static_cast<double>(segment) /
            static_cast<double>(arcSegments));
    waypoints[count++] = end;
    return count;
}

glm::dquat ClosestSquarePlacementOrientation(const glm::dquat& goalBoxOrientation,
    const glm::dquat& boxRotationOffset, const glm::dquat& currentTcpOrientation)
{
    glm::dquat bestTcpOrientation = goalBoxOrientation * glm::inverse(boxRotationOffset);
    double bestAlignment = -1.0;
    for (int quarterTurn = 0; quarterTurn < 4; ++quarterTurn)
    {
        const glm::dquat equivalentBoxOrientation = glm::angleAxis(
            static_cast<double>(quarterTurn) * glm::half_pi<double>(), glm::dvec3{0.0, 1.0, 0.0}) * goalBoxOrientation;
        const glm::dquat candidateTcpOrientation = glm::normalize(
            equivalentBoxOrientation * glm::inverse(boxRotationOffset));
        const double alignment = std::abs(glm::dot(candidateTcpOrientation, currentTcpOrientation));
        if (alignment > bestAlignment)
        {
            bestAlignment = alignment;
            bestTcpOrientation = candidateTcpOrientation;
        }
    }
    return bestTcpOrientation;
}
}

void RobotPanel::DrawContents(robotics::backends::simulation::SimRobotController& controller,
    robotics::IGripperController& gripper, bool boxGrasped,
    const robotics::CartesianPose& graspBoxPoseInBase,
    const robotics::CartesianPose& placementPoseInBase)
{
    const auto& state = controller.GetStateView();
    const auto makeBoxTarget = [&](const robotics::CartesianPose& boxPose, double heightOffset,
        const std::array<double, 4>& orientation)
    {
        robotics::CartesianPose target = boxPose;
        target.orientationXyzw = orientation;
        target.positionMeters[1] += heightOffset;
        return target;
    };
    const auto makePlacementTarget = [&](double heightOffset)
    {
        robotics::CartesianPose target{};
        const glm::dquat targetBoxOrientation = Orientation(placementBoxOrientationXyzw_);
        const glm::dquat boxRotationOffset{boxRotationOffsetInTool_[3], boxRotationOffsetInTool_[0],
            boxRotationOffsetInTool_[1], boxRotationOffsetInTool_[2]};
        const glm::dquat targetTcpOrientation = glm::normalize(targetBoxOrientation * glm::inverse(boxRotationOffset));
        const glm::dvec3 boxOffset = targetTcpOrientation *
            glm::dvec3{boxOffsetInTool_[0], boxOffsetInTool_[1], boxOffsetInTool_[2]};
        target.orientationXyzw = QuaternionXyzw(targetTcpOrientation);
        target.positionMeters = {
            placementPoseInBase.positionMeters[0] - boxOffset.x,
            placementPoseInBase.positionMeters[1] + heightOffset - boxOffset.y,
            placementPoseInBase.positionMeters[2] - boxOffset.z};
        return target;
    };
    const auto makeAttachedBoxTarget = [&](const robotics::CartesianPose& boxPose, double heightOffset,
        const glm::dquat& tcpOrientation)
    {
        robotics::CartesianPose target{};
        const glm::dquat boxRotationOffset{boxRotationOffsetInTool_[3], boxRotationOffsetInTool_[0],
            boxRotationOffsetInTool_[1], boxRotationOffsetInTool_[2]};
        const glm::dvec3 boxOffset = tcpOrientation *
            glm::dvec3{boxOffsetInTool_[0], boxOffsetInTool_[1], boxOffsetInTool_[2]};
        target.orientationXyzw = QuaternionXyzw(tcpOrientation);
        target.positionMeters = {boxPose.positionMeters[0] - boxOffset.x,
            boxPose.positionMeters[1] + heightOffset - boxOffset.y,
            boxPose.positionMeters[2] - boxOffset.z};
        return target;
    };
    const auto makeAttachedBoxPose = [&](const robotics::CartesianPose& boxPose, double heightOffset)
    {
        const glm::dquat boxRotationOffset{boxRotationOffsetInTool_[3], boxRotationOffsetInTool_[0],
            boxRotationOffsetInTool_[1], boxRotationOffsetInTool_[2]};
        const glm::dquat tcpOrientation = glm::normalize(
            Orientation(boxPose.orientationXyzw) * glm::inverse(boxRotationOffset));
        return makeAttachedBoxTarget(boxPose, heightOffset, tcpOrientation);
    };
    const auto moveTo = [&](const robotics::CartesianPose& pose)
    {
        lastResult_ = controller.MoveLinear({pose, kLinearVelocityMetersPerSecond, kAngularVelocityRadiansPerSecond,
            kLinearAccelerationMetersPerSecondSquared, kAngularAccelerationRadiansPerSecondSquared});
        hasResult_ = true;
        return lastResult_.Ok();
    };
    const auto movePath = [&](const std::array<robotics::CartesianPose, 6>& poses, std::size_t count)
    {
        robotics::LinearPathMoveCommand command;
        command.targetPoses.assign(poses.begin(), poses.begin() + count);
        command.maxLinearVelocityMetersPerSecond = kLinearVelocityMetersPerSecond;
        command.maxAngularVelocityRadiansPerSecond = kAngularVelocityRadiansPerSecond;
        command.maxLinearAccelerationMetersPerSecondSquared = kLinearAccelerationMetersPerSecondSquared;
        command.maxAngularAccelerationRadiansPerSecondSquared = kAngularAccelerationRadiansPerSecondSquared;
        lastResult_ = controller.MoveLinearPath(command);
        hasResult_ = true;
        return lastResult_.Ok();
    };
    const auto commandGripper = [&](std::uint8_t position)
    {
        robotics::GripperCommand command;
        command.positionRequest = position;
        command.speedRequest = 255;
        command.forceRequest = 128;
        lastResult_ = gripper.Command(command);
        hasResult_ = true;
        return lastResult_.Ok();
    };
    const auto alignWristOnly = [&](const std::array<double, 4>& targetOrientation, bool carryingBox)
    {
        const std::optional<std::array<double, 3>> attachedOffset = carryingBox
            ? std::optional<std::array<double, 3>>{boxOffsetInTool_}
            : std::nullopt;
        const auto plan = detail::PlanWristOnlyTarget(
            controller.GetSpecification(), state, targetOrientation, attachedOffset);
        if (!plan)
        {
            lastResult_ = {robotics::ErrorCode::Unsupported,
                "RobotPanel: wrist alignment needs another axis, exceeds a J6 limit, or moves the attached box over 20 mm"};
            hasResult_ = true;
            return false;
        }
        lastResult_ = controller.MoveJoint({plan->jointPositionRadians, 1.0, 1.0});
        hasResult_ = true;
        return lastResult_.Ok();
    };
    // Viewer가 겹친 틱을 취소하면 Controller는 직전 관절각에서 Idle로 멈추고 충돌 원인을 남긴다. 여기서는 하강을 이어가지 않고 저장한 높이로 후퇴한다.
    const bool collisionStopped = state.mode == robotics::RobotMode::Idle &&
        state.faultCode == static_cast<std::uint32_t>(robotics::ErrorCode::EnvironmentContact) &&
        stage_ != Stage::Ready && stage_ != Stage::Complete && stage_ != Stage::Failed && stage_ != Stage::Recovering;

    if (!orientationReady_ && state.valid && state.tcpPoseValid)
    {
        const glm::dquat current = TcpOrientation(state);
        // GLB에서 그리퍼의 Y축은 손가락 면에 수직인 접근 방향이다. X축 둘레로 90도 돌리면 손가락 면이 수직이고 접근 축이 바닥을 향한다.
        const glm::dquat faceDown = glm::angleAxis(glm::half_pi<double>(), glm::dvec3{1.0, 0.0, 0.0}) * current;
        graspOrientationXyzw_ = {faceDown.x, faceDown.y, faceDown.z, faceDown.w};
        orientationReady_ = true;
    }

    if (collisionStopped)
    {
        stage_ = moveTo(recoveryPose_) ? Stage::Recovering : Stage::Failed;
    }
    else if (stage_ == Stage::UnwindingBeforeTask && state.mode == robotics::RobotMode::Idle)
    {
        if (state.jointPositionRadians.empty() ||
            std::abs(state.jointPositionRadians.back()) > kJ6UnwindToleranceRadians)
        {
            lastResult_ = {robotics::ErrorCode::Fault, "RobotPanel: J6 did not return to the 0 degree unwind position"};
            hasResult_ = true;
            stage_ = Stage::Failed;
        }
        else
        {
            const auto approach = makeBoxTarget(graspBoxPoseInBase, kApproachHeightMeters,
                graspOrientationXyzw_);
            stage_ = moveTo(approach) ? Stage::MovingAbovePickup : Stage::Failed;
        }
    }
    else if (stage_ == Stage::MovingAbovePickup && state.mode == robotics::RobotMode::Idle)
    {
        const double boxYawRadians = glm::radians(YawDegrees(graspBoxPoseInBase.orientationXyzw));
        const glm::dquat desiredPickupOrientation = glm::normalize(
            glm::angleAxis(boxYawRadians, glm::dvec3{0.0, 1.0, 0.0}) * Orientation(graspOrientationXyzw_));
        pickupOrientationXyzw_ = QuaternionXyzw(desiredPickupOrientation);
        recoveryPose_ = makeBoxTarget(graspBoxPoseInBase, kApproachHeightMeters,
            QuaternionXyzw(TcpOrientation(state)));
        stage_ = alignWristOnly(pickupOrientationXyzw_, false)
            ? Stage::AligningAbovePickup : Stage::Failed;
    }
    else if (stage_ == Stage::AligningAbovePickup && state.mode == robotics::RobotMode::Idle)
    {
        const auto target = makeBoxTarget(graspBoxPoseInBase, kGraspClearanceMeters, pickupOrientationXyzw_);
        stage_ = moveTo(target) ? Stage::MovingDownToPickup : Stage::Failed;
    }
    else if (stage_ == Stage::MovingDownToPickup && state.mode == robotics::RobotMode::Idle)
    {
        stage_ = commandGripper(255) ? Stage::Closing : Stage::Failed;
    }
    else if (stage_ == Stage::Closing && boxGrasped && state.tcpPoseValid)
    {
        const glm::dvec3 boxPosition{graspBoxPoseInBase.positionMeters[0], graspBoxPoseInBase.positionMeters[1], graspBoxPoseInBase.positionMeters[2]};
        const glm::dvec3 tcpPosition{state.tcpPose.positionMeters[0], state.tcpPose.positionMeters[1], state.tcpPose.positionMeters[2]};
        // Constraint는 파지 순간의 물체와 그리퍼 상대 자세를 유지한다. 이 차이를 TCP의 Local 좌표로 저장하면 방향이 바뀌어도 목표 위치에 상자 중심을 맞출 수 있다.
        const glm::dvec3 localOffset = glm::inverse(TcpOrientation(state)) * (boxPosition - tcpPosition);
        boxOffsetInTool_ = {localOffset.x, localOffset.y, localOffset.z};
        const glm::dquat localRotation = glm::inverse(TcpOrientation(state)) * Orientation(graspBoxPoseInBase.orientationXyzw);
        boxRotationOffsetInTool_ = QuaternionXyzw(localRotation);
        graspedBoxPoseInBase_ = graspBoxPoseInBase;
        recoveryPose_ = makeAttachedBoxPose(graspBoxPoseInBase, kApproachHeightMeters);
        stage_ = moveTo(recoveryPose_) ? Stage::Lifting : Stage::Failed;
    }
    else if (stage_ == Stage::Closing && !boxGrasped &&
        gripper.GetState().mode == robotics::GripperMode::Idle &&
        gripper.GetState().objectStatus == robotics::GripperObjectStatus::AtRequestedPosition)
    {
        taskSucceeded_ = false;
        if (commandGripper(0) && moveTo(recoveryPose_))
            stage_ = Stage::Recovering;
        else
            stage_ = Stage::Failed;
    }
    else if (stage_ == Stage::Lifting && state.mode == robotics::RobotMode::Idle)
    {
        recoveryPose_ = makeAttachedBoxPose(graspedBoxPoseInBase_, kTransitBoxHeightMeters);
        stage_ = moveTo(recoveryPose_) ? Stage::TransitingToPlacement : Stage::Failed;
    }
    else if (stage_ == Stage::TransitingToPlacement && state.mode == robotics::RobotMode::Idle)
    {
        if (transitWaypointCount_ == 0)
        {
            robotics::CartesianPose sourceBox = graspedBoxPoseInBase_;
            sourceBox.positionMeters[1] += kTransitBoxHeightMeters;
            robotics::CartesianPose destinationBox = placementPoseInBase;
            destinationBox.positionMeters[1] += kTransitBoxHeightMeters;
            destinationBox.orientationXyzw = graspBoxPoseInBase.orientationXyzw;
            transitWaypointCount_ = BuildTransitWaypoints(sourceBox, destinationBox, transitWaypoints_);
            const glm::dquat tcpOrientation = TcpOrientation(state);
            const glm::dvec3 boxOffset = tcpOrientation *
                glm::dvec3{boxOffsetInTool_[0], boxOffsetInTool_[1], boxOffsetInTool_[2]};
            for (std::size_t waypoint = 0; waypoint < transitWaypointCount_; ++waypoint)
            {
                transitWaypoints_[waypoint].positionMeters[0] -= boxOffset.x;
                transitWaypoints_[waypoint].positionMeters[1] -= boxOffset.y;
                transitWaypoints_[waypoint].positionMeters[2] -= boxOffset.z;
                transitWaypoints_[waypoint].orientationXyzw = QuaternionXyzw(tcpOrientation);
            }
        }
        stage_ = movePath(transitWaypoints_, transitWaypointCount_)
            ? Stage::TransitPathRunning : Stage::Failed;
    }
    else if (stage_ == Stage::TransitPathRunning && state.mode == robotics::RobotMode::Idle)
    {
        const glm::dquat boxRotationOffset{boxRotationOffsetInTool_[3], boxRotationOffsetInTool_[0],
            boxRotationOffsetInTool_[1], boxRotationOffsetInTool_[2]};
        const glm::dquat currentTcp = TcpOrientation(state);
        const glm::dquat chosenTcp = ClosestSquarePlacementOrientation(
            Orientation(placementPoseInBase.orientationXyzw), boxRotationOffset, currentTcp);
        placementBoxOrientationXyzw_ = QuaternionXyzw(chosenTcp * boxRotationOffset);
        recoveryPose_ = makeAttachedBoxTarget(placementPoseInBase, kTransitBoxHeightMeters, currentTcp);
        stage_ = moveTo(recoveryPose_) ? Stage::MovingToPlacementOverhead : Stage::Failed;
    }
    else if (stage_ == Stage::MovingToPlacementOverhead && state.mode == robotics::RobotMode::Idle)
    {
        stage_ = alignWristOnly(QuaternionXyzw(Orientation(placementBoxOrientationXyzw_) *
            glm::inverse(glm::dquat{boxRotationOffsetInTool_[3], boxRotationOffsetInTool_[0],
                boxRotationOffsetInTool_[1], boxRotationOffsetInTool_[2]})), true)
            ? Stage::AligningAbovePlacement : Stage::Failed;
    }
    else if (stage_ == Stage::AligningAbovePlacement && state.mode == robotics::RobotMode::Idle)
    {
        recoveryPose_ = makePlacementTarget(kApproachHeightMeters);
        stage_ = moveTo(recoveryPose_) ? Stage::MovingAbovePlacement : Stage::Failed;
    }
    else if (stage_ == Stage::MovingAbovePlacement && state.mode == robotics::RobotMode::Idle)
    {
        stage_ = moveTo(makePlacementTarget(0.0)) ? Stage::MovingDownToPlacement : Stage::Failed;
    }
    else if (stage_ == Stage::MovingDownToPlacement && state.mode == robotics::RobotMode::Idle)
    {
        stage_ = commandGripper(0) ? Stage::Opening : Stage::Failed;
    }
    else if (stage_ == Stage::Opening && !boxGrasped)
    {
        placementReleased_ = true;
        stage_ = moveTo(recoveryPose_) ? Stage::Retreating : Stage::Failed;
    }
    else if (stage_ == Stage::Retreating && state.mode == robotics::RobotMode::Idle)
    {
        if (placementReleased_ && taskSucceeded_ && !boxGrasped)
        {
            auto unwindTarget = state.jointPositionRadians;
            if (unwindTarget.empty())
            {
                stage_ = Stage::Failed;
                autoLoopEnabled_ = false;
            }
            else
            {
                unwindTarget.back() = 0.0;
                robotics::JointMoveCommand unwindCommand;
                unwindCommand.targetPositionRadians = std::move(unwindTarget);
                unwindCommand.preserveJointTurns = true;
                lastResult_ = controller.MoveJoint(unwindCommand);
                hasResult_ = true;
                stage_ = lastResult_.Ok() ? Stage::UnwindingWrist : Stage::Failed;
            }
        }
        else
            stage_ = Stage::UnwindingWrist;
    }
    else if (stage_ == Stage::UnwindingWrist && state.mode == robotics::RobotMode::Idle)
    {
        const bool wristUnwound = !state.jointPositionRadians.empty() &&
            std::abs(state.jointPositionRadians.back()) <= kJ6UnwindToleranceRadians;
        if (!wristUnwound)
        {
            lastResult_ = {robotics::ErrorCode::Fault, "RobotPanel: J6 did not return to the 0 degree unwind position"};
            hasResult_ = true;
        }
        missionSucceeded_ = wristUnwound && taskSucceeded_ && placementReleased_ &&
            BoxCornersFitPlacement(graspBoxPoseInBase, placementPoseInBase);
        stage_ = missionSucceeded_ ? Stage::Complete : Stage::Failed;
        if (missionSucceeded_)
        {
            ++completedMissionCount_;
            missionSuccessEventPending_ = true;
        }
        else
            autoLoopEnabled_ = false;
    }

    if (!collisionStopped && stage_ == Stage::Recovering && state.mode == robotics::RobotMode::Idle)
    {
        stage_ = Stage::Retreating;
    }
    if (stage_ == Stage::Failed)
        autoLoopEnabled_ = false;

    ImGui::SeparatorText("Pick and place");
    if (state.valid && state.tcpPoseValid)
        ImGui::Text("TCP: %.3f, %.3f, %.3f m", state.tcpPose.positionMeters[0], state.tcpPose.positionMeters[1], state.tcpPose.positionMeters[2]);
    ImGui::Text("Box X/Z: %.3f, %.3f m", graspBoxPoseInBase.positionMeters[0], graspBoxPoseInBase.positionMeters[2]);
    ImGui::Text("Box yaw: %.1f deg", YawDegrees(graspBoxPoseInBase.orientationXyzw));
    ImGui::Text("Goal X/Z: %.3f, %.3f m", placementPoseInBase.positionMeters[0], placementPoseInBase.positionMeters[2]);
    ImGui::Text("Goal yaw: %.1f deg", YawDegrees(placementPoseInBase.orientationXyzw));
    ImGui::Text("TCP ceilings: %.1f m/s, %.1f rad/s", kLinearVelocityMetersPerSecond, kAngularVelocityRadiansPerSecond);
    const auto& specification = controller.GetSpecification();
    if (ImGui::BeginTable("Joint positions", 3, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
    {
        ImGui::TableSetupColumn("Joint", ImGuiTableColumnFlags_WidthFixed, 42.0F);
        ImGui::TableSetupColumn("Min / max", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Current", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();
        for (std::size_t joint = 0; joint < specification.jointCount; ++joint)
        {
            ImGui::TableNextRow();
            ImGui::TableSetColumnIndex(0);
            const std::string_view jointName = specification.joints[joint].name;
            ImGui::TextUnformatted(jointName.data(), jointName.data() + jointName.size());
            ImGui::TableSetColumnIndex(1);
            ImGui::Text("%.1f / %.1f deg",
                glm::degrees(specification.joints[joint].minPositionRadians),
                glm::degrees(specification.joints[joint].maxPositionRadians));
            ImGui::TableSetColumnIndex(2);
            if (joint < state.jointPositionRadians.size())
                ImGui::Text("%.1f deg", glm::degrees(state.jointPositionRadians[joint]));
            else
                ImGui::TextUnformatted("--");
        }
        ImGui::EndTable();
    }
    ImGui::TextUnformatted("Angle values are current joint positions, not travel percentages.");
    const bool canStart = !boxGrasped &&
        (stage_ == Stage::Ready || stage_ == Stage::Complete || stage_ == Stage::Failed);
    // 첫 임무는 버튼으로 시작하고 성공 뒤에는 Viewer가 상자를 옮기고 Ready로 돌려놓아 자동으로 다음 임무를 시작한다.
    const bool startRequested = canStart && orientationReady_ &&
        (ImGui::Button("Run random pick and place") || (autoLoopEnabled_ && stage_ == Stage::Ready));
    if (startRequested)
    {
        taskSucceeded_ = true;
        placementReleased_ = false;
        missionSucceeded_ = false;
        transitWaypointCount_ = 0;
        autoLoopEnabled_ = true;
        auto unwindTarget = state.jointPositionRadians;
        if (unwindTarget.empty())
        {
            lastResult_ = {robotics::ErrorCode::InvalidCommand, "RobotPanel: current joint positions are unavailable"};
            hasResult_ = true;
            stage_ = Stage::Failed;
        }
        else
        {
            unwindTarget.back() = 0.0;
            robotics::JointMoveCommand unwindCommand;
            unwindCommand.targetPositionRadians = std::move(unwindTarget);
            unwindCommand.preserveJointTurns = true;
            lastResult_ = controller.MoveJoint(unwindCommand);
            hasResult_ = true;
            stage_ = lastResult_.Ok() ? Stage::UnwindingBeforeTask : Stage::Failed;
        }
    }
    if (ImGui::Button("Stop robot"))
    {
        lastResult_ = controller.Stop();
        hasResult_ = true;
        stage_ = Stage::Ready;
        autoLoopEnabled_ = false;
    }
    const char* stageName = stage_ == Stage::Complete ? "Complete" : stage_ == Stage::Failed ? "Failed" : stage_ == Stage::Recovering ? "Retracting after collision" : stage_ == Stage::UnwindingWrist ? "Unwinding J6" : "Running / ready";
    ImGui::Text("Task: %s", stageName);
    ImGui::Text("Mission success: %s", missionSucceeded_ ? "YES" : "NO");
    ImGui::Text("Successful missions: %llu", static_cast<unsigned long long>(completedMissionCount_));
    ImGui::Text("Automatic repeat: %s", autoLoopEnabled_ ? "ON" : "OFF");
    if (state.faultCode == static_cast<std::uint32_t>(robotics::ErrorCode::EnvironmentContact))
        ImGui::TextWrapped("A collision stopped the arm at its last safe pose. The task is retracting to the previous safe height.");
    if (hasResult_)
    {
        ImGui::Text("Last request: %s", lastResult_.Ok() ? "accepted" : "failed");
        if (!lastResult_.message.empty())
            ImGui::TextWrapped("%s", lastResult_.message.c_str());
    }
}

bool RobotPanel::ConsumeMissionSuccessEvent() noexcept
{
    const bool result = missionSuccessEventPending_;
    missionSuccessEventPending_ = false;
    return result;
}

void RobotPanel::PrepareNextTask() noexcept
{
    stage_ = Stage::Ready;
    missionSucceeded_ = false;
    placementReleased_ = false;
    taskSucceeded_ = true;
    transitWaypointCount_ = 0;
}
}
