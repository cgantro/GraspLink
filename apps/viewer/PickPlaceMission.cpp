#include "PickPlaceMission.h"
#include "robotics/planning/WristAlignmentPlanner.h"

#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <string_view>
#include <utility>

namespace grasplink::viewer
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

glm::dquat CaptureAttachedBoxOffset(const robotics::CartesianPose& boxPose,
    const robotics::RobotState& state, std::array<double, 3>& positionOffset,
    std::array<double, 4>& rotationOffset)
{
    const glm::dquat tcpOrientation = TcpOrientation(state);
    const glm::dquat inverseTcp = glm::inverse(tcpOrientation);
    const glm::dvec3 boxPosition{boxPose.positionMeters[0], boxPose.positionMeters[1], boxPose.positionMeters[2]};
    const glm::dvec3 tcpPosition{state.tcpPose.positionMeters[0], state.tcpPose.positionMeters[1], state.tcpPose.positionMeters[2]};
    const glm::dvec3 localPosition = inverseTcp * (boxPosition - tcpPosition);
    positionOffset = {localPosition.x, localPosition.y, localPosition.z};
    rotationOffset = QuaternionXyzw(inverseTcp * Orientation(boxPose.orientationXyzw));
    return tcpOrientation;
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

robotics::CartesianPose MakeBoxTarget(const robotics::CartesianPose& boxPose, double heightOffset,
    const std::array<double, 4>& orientation)
{
    robotics::CartesianPose target = boxPose;
    target.orientationXyzw = orientation;
    target.positionMeters[1] += heightOffset;
    return target;
}

robotics::CartesianPose MakeAttachedBoxTarget(const robotics::CartesianPose& boxPose, double heightOffset,
    const glm::dquat& tcpOrientation, const std::array<double, 3>& boxOffsetInTool)
{
    const glm::dvec3 boxOffset = tcpOrientation *
        glm::dvec3{boxOffsetInTool[0], boxOffsetInTool[1], boxOffsetInTool[2]};
    robotics::CartesianPose target{};
    target.orientationXyzw = QuaternionXyzw(tcpOrientation);
    target.positionMeters = {boxPose.positionMeters[0] - boxOffset.x,
        boxPose.positionMeters[1] + heightOffset - boxOffset.y,
        boxPose.positionMeters[2] - boxOffset.z};
    return target;
}

robotics::CartesianPose MakePlacementTarget(const robotics::CartesianPose& placementPose,
    const std::array<double, 4>& placementBoxOrientation, const std::array<double, 4>& boxRotationOffsetInTool,
    const std::array<double, 3>& boxOffsetInTool, double heightOffset)
{
    const glm::dquat boxRotationOffset{boxRotationOffsetInTool[3], boxRotationOffsetInTool[0],
        boxRotationOffsetInTool[1], boxRotationOffsetInTool[2]};
    const glm::dquat tcpOrientation = glm::normalize(
        Orientation(placementBoxOrientation) * glm::inverse(boxRotationOffset));
    return MakeAttachedBoxTarget(placementPose, heightOffset, tcpOrientation, boxOffsetInTool);
}

robotics::CartesianPose MakeAttachedBoxPose(const robotics::CartesianPose& boxPose, double heightOffset,
    const std::array<double, 4>& boxRotationOffsetInTool, const std::array<double, 3>& boxOffsetInTool)
{
    const glm::dquat rotationOffset{boxRotationOffsetInTool[3], boxRotationOffsetInTool[0],
        boxRotationOffsetInTool[1], boxRotationOffsetInTool[2]};
    const glm::dquat tcpOrientation = glm::normalize(
        Orientation(boxPose.orientationXyzw) * glm::inverse(rotationOffset));
    return MakeAttachedBoxTarget(boxPose, heightOffset, tcpOrientation, boxOffsetInTool);
}

robotics::Result MoveTo(const robotics::CartesianPose& pose, robotics::IRobotController& controller)
{
    return controller.MoveLinear({pose, kLinearVelocityMetersPerSecond, kAngularVelocityRadiansPerSecond,
        kLinearAccelerationMetersPerSecondSquared, kAngularAccelerationRadiansPerSecondSquared});
}

robotics::Result MovePath(const std::array<robotics::CartesianPose, 6>& poses, std::size_t count,
    robotics::IRobotController& controller)
{
    robotics::LinearPathMoveCommand command;
    command.targetPoses.assign(poses.begin(), poses.begin() + count);
    command.maxLinearVelocityMetersPerSecond = kLinearVelocityMetersPerSecond;
    command.maxAngularVelocityRadiansPerSecond = kAngularVelocityRadiansPerSecond;
    command.maxLinearAccelerationMetersPerSecondSquared = kLinearAccelerationMetersPerSecondSquared;
    command.maxAngularAccelerationRadiansPerSecondSquared = kAngularAccelerationRadiansPerSecondSquared;
    return controller.MoveLinearPath(command);
}

robotics::Result CommandGripper(std::uint8_t position, robotics::IGripperController& gripper)
{
    robotics::GripperCommand command;
    command.positionRequest = position;
    command.speedRequest = 255;
    command.forceRequest = 128;
    return gripper.Command(command);
}

robotics::Result AlignWristOnly(const robotics::models::RobotSpecification& specification,
    const robotics::RobotState& state, const std::array<double, 4>& targetOrientation, bool carryingBox,
    const std::array<double, 3>& boxOffsetInTool, robotics::IRobotController& controller)
{
    const std::optional<std::array<double, 3>> attachedOffset = carryingBox
        ? std::optional<std::array<double, 3>>{boxOffsetInTool}
        : std::nullopt;
    const auto plan = robotics::planning::PlanWristOnlyTarget(
        specification, state, targetOrientation, attachedOffset);
    if (!plan)
    {
        return {robotics::ErrorCode::Unsupported,
            "RobotPanel: wrist alignment needs another axis, exceeds a J6 limit, or moves the attached box over 20 mm"};
    }
    return controller.MoveJoint({plan->jointPositionRadians, 1.0, 1.0});
}
}


PickPlaceMission::PickPlaceMission(const robotics::models::RobotSpecification& specification) noexcept
    : specification_(specification)
{
}

bool PickPlaceMission::SetResult(robotics::Result result)
{
    lastResult_ = std::move(result);
    hasResult_ = true;
    return lastResult_.Ok();
}

void PickPlaceMission::SetStageFromResult(robotics::Result result, Stage next)
{
    stage_ = SetResult(std::move(result)) ? next : Stage::Failed;
}

robotics::Result PickPlaceMission::RequestJ6Unwind(const robotics::RobotState& state,
    robotics::IRobotController& controller) const
{
    if (state.jointPositionRadians.empty())
        return {robotics::ErrorCode::InvalidCommand, "RobotPanel: current joint positions are unavailable"};

    robotics::JointMoveCommand command;
    command.targetPositionRadians = state.jointPositionRadians;
    command.targetPositionRadians.back() = 0.0;
    command.preserveJointTurns = true;
    return controller.MoveJoint(command);
}

void PickPlaceMission::Update(const robotics::RobotState& state, robotics::IRobotController& controller,
    robotics::IGripperController& gripper, bool boxGrasped,
    const robotics::CartesianPose& graspBoxPoseInBase,
    const robotics::CartesianPose& placementPoseInBase)
{
    const bool idle = state.mode == robotics::RobotMode::Idle;
    bool gripperCloseFinished = false;
    if (!taskPaused_ && stage_ == Stage::Closing && !boxGrasped)
    {
        const auto gripperState = gripper.GetState();
        gripperCloseFinished = gripperState.mode == robotics::GripperMode::Idle &&
            gripperState.objectStatus == robotics::GripperObjectStatus::AtRequestedPosition;
    }
    // Viewer가 겹친 틱을 취소하면 Controller는 직전 관절각에서 Idle로 멈추고 충돌 원인을 남긴다. 여기서는 하강을 이어가지 않고 저장한 높이로 후퇴한다.
    const bool collisionStopped = idle &&
        state.errorCode == robotics::ErrorCode::EnvironmentContact &&
        stage_ != Stage::Ready && stage_ != Stage::Complete && stage_ != Stage::Failed && stage_ != Stage::Recovering;

    if (!orientationReady_ && state.valid && state.tcpPoseValid)
    {
        const glm::dquat current = TcpOrientation(state);
        // GLB에서 그리퍼의 Y축은 손가락 면에 수직인 접근 방향이다. X축 둘레로 90도 돌리면 손가락 면이 수직이고 접근 축이 바닥을 향한다.
        const glm::dquat faceDown = glm::angleAxis(glm::half_pi<double>(), glm::dvec3{1.0, 0.0, 0.0}) * current;
        graspOrientationXyzw_ = {faceDown.x, faceDown.y, faceDown.z, faceDown.w};
        orientationReady_ = true;
    }

    if (!taskPaused_ && collisionStopped)
    {
        SetStageFromResult(MoveTo(recoveryPose_, controller), Stage::Recovering);
    }
    else if (stage_ == Stage::UnwindingBeforeTask && idle)
    {
        if (state.jointPositionRadians.empty() ||
            std::abs(state.jointPositionRadians.back()) > kJ6UnwindToleranceRadians)
        {
            SetResult({robotics::ErrorCode::Fault,
                "RobotPanel: J6 did not return to the 0 degree unwind position"});
            stage_ = Stage::Failed;
        }
        else
        {
            const auto approach = MakeBoxTarget(graspBoxPoseInBase, kApproachHeightMeters,
                graspOrientationXyzw_);
            SetStageFromResult(MoveTo(approach, controller), Stage::MovingAbovePickup);
        }
    }
    else if (stage_ == Stage::MovingAbovePickup && idle)
    {
        const double boxYawRadians = glm::radians(YawDegrees(graspBoxPoseInBase.orientationXyzw));
        const glm::dquat desiredPickupOrientation = glm::normalize(
            glm::angleAxis(boxYawRadians, glm::dvec3{0.0, 1.0, 0.0}) * Orientation(graspOrientationXyzw_));
        pickupOrientationXyzw_ = QuaternionXyzw(desiredPickupOrientation);
        recoveryPose_ = MakeBoxTarget(graspBoxPoseInBase, kApproachHeightMeters,
            QuaternionXyzw(TcpOrientation(state)));
        SetStageFromResult(AlignWristOnly(specification_, state, pickupOrientationXyzw_, false,
            boxOffsetInTool_, controller), Stage::AligningAbovePickup);
    }
    else if (stage_ == Stage::AligningAbovePickup && idle)
    {
        const auto target = MakeBoxTarget(graspBoxPoseInBase, kGraspClearanceMeters, pickupOrientationXyzw_);
        SetStageFromResult(MoveTo(target, controller), Stage::MovingDownToPickup);
    }
    else if (stage_ == Stage::MovingDownToPickup && idle)
    {
        SetStageFromResult(CommandGripper(255, gripper), Stage::Closing);
    }
    else if (!taskPaused_ && stage_ == Stage::Closing && boxGrasped && state.tcpPoseValid)
    {
        // Constraint는 파지 순간의 물체와 그리퍼 상대 자세를 유지한다. 이 차이를 TCP의 Local 좌표로 저장하면 방향이 바뀌어도 목표 위치에 상자 중심을 맞출 수 있다.
        CaptureAttachedBoxOffset(graspBoxPoseInBase, state, boxOffsetInTool_, boxRotationOffsetInTool_);
        graspedBoxPoseInBase_ = graspBoxPoseInBase;
        recoveryPose_ = MakeAttachedBoxPose(graspBoxPoseInBase, kApproachHeightMeters,
            boxRotationOffsetInTool_, boxOffsetInTool_);
        SetStageFromResult(MoveTo(recoveryPose_, controller), Stage::Lifting);
    }
    else if (gripperCloseFinished)
    {
        taskSucceeded_ = false;
        if (SetResult(CommandGripper(0, gripper)) && SetResult(MoveTo(recoveryPose_, controller)))
            stage_ = Stage::Recovering;
        else
            stage_ = Stage::Failed;
    }
    else if (stage_ == Stage::Lifting && idle)
    {
        recoveryPose_ = MakeAttachedBoxPose(graspedBoxPoseInBase_, kTransitBoxHeightMeters,
            boxRotationOffsetInTool_, boxOffsetInTool_);
        SetStageFromResult(MoveTo(recoveryPose_, controller), Stage::TransitingToPlacement);
    }
    else if (stage_ == Stage::RaisingAfterResume && idle)
    {
        stage_ = Stage::TransitingToPlacement;
    }
    else if (stage_ == Stage::TransitingToPlacement && idle)
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
        SetStageFromResult(MovePath(transitWaypoints_, transitWaypointCount_, controller), Stage::TransitPathRunning);
    }
    else if (stage_ == Stage::TransitPathRunning && idle)
    {
        const glm::dquat boxRotationOffset{boxRotationOffsetInTool_[3], boxRotationOffsetInTool_[0],
            boxRotationOffsetInTool_[1], boxRotationOffsetInTool_[2]};
        const glm::dquat currentTcp = TcpOrientation(state);
        const glm::dquat chosenTcp = ClosestSquarePlacementOrientation(
            Orientation(placementPoseInBase.orientationXyzw), boxRotationOffset, currentTcp);
        placementBoxOrientationXyzw_ = QuaternionXyzw(chosenTcp * boxRotationOffset);
        recoveryPose_ = MakeAttachedBoxTarget(placementPoseInBase, kTransitBoxHeightMeters, currentTcp,
            boxOffsetInTool_);
        SetStageFromResult(MoveTo(recoveryPose_, controller), Stage::MovingToPlacementOverhead);
    }
    else if (stage_ == Stage::MovingToPlacementOverhead && idle)
    {
        const glm::dquat boxRotationOffset{boxRotationOffsetInTool_[3], boxRotationOffsetInTool_[0],
            boxRotationOffsetInTool_[1], boxRotationOffsetInTool_[2]};
        SetStageFromResult(
            AlignWristOnly(specification_, state,
                QuaternionXyzw(Orientation(placementBoxOrientationXyzw_) *
                    glm::inverse(boxRotationOffset)), true, boxOffsetInTool_, controller),
            Stage::AligningAbovePlacement);
        if (stage_ == Stage::Failed)
        {
            const robotics::Result wristOnlyFailure = lastResult_;
            recoveryPose_ = MakePlacementTarget(placementPoseInBase, placementBoxOrientationXyzw_,
                boxRotationOffsetInTool_, boxOffsetInTool_, kTransitBoxHeightMeters);
            if (SetResult(MoveTo(recoveryPose_, controller)))
                stage_ = Stage::AligningAbovePlacement;
            else
            {
                lastResult_.message = "RobotPanel: J6-only alignment failed (" + wristOnlyFailure.message +
                    "); compensated TCP alignment also failed: " + lastResult_.message;
            }
        }
    }
    else if (stage_ == Stage::AligningAbovePlacement && idle)
    {
        recoveryPose_ = MakePlacementTarget(placementPoseInBase, placementBoxOrientationXyzw_, boxRotationOffsetInTool_, boxOffsetInTool_, kApproachHeightMeters);
        SetStageFromResult(MoveTo(recoveryPose_, controller), Stage::MovingAbovePlacement);
    }
    else if (stage_ == Stage::MovingAbovePlacement && idle)
    {
        SetStageFromResult(
            MoveTo(MakePlacementTarget(placementPoseInBase, placementBoxOrientationXyzw_,
                boxRotationOffsetInTool_, boxOffsetInTool_, 0.0), controller),
            Stage::MovingDownToPlacement);
    }
    else if (stage_ == Stage::MovingDownToPlacement && idle)
    {
        SetStageFromResult(CommandGripper(0, gripper), Stage::Opening);
    }
    else if (!taskPaused_ && stage_ == Stage::Opening && !boxGrasped)
    {
        placementReleased_ = true;
        SetStageFromResult(MoveTo(recoveryPose_, controller), Stage::Retreating);
    }
    else if (stage_ == Stage::Retreating && idle)
    {
        if (placementReleased_ && taskSucceeded_ && !boxGrasped)
        {
            SetStageFromResult(RequestJ6Unwind(state, controller), Stage::UnwindingWrist);
            if (stage_ == Stage::Failed)
                autoLoopEnabled_ = false;
        }
        else
            stage_ = Stage::UnwindingWrist;
    }
    else if (stage_ == Stage::UnwindingWrist && idle)
    {
        const bool wristUnwound = !state.jointPositionRadians.empty() &&
            std::abs(state.jointPositionRadians.back()) <= kJ6UnwindToleranceRadians;
        if (!wristUnwound)
        {
            SetResult({robotics::ErrorCode::Fault,
                "RobotPanel: J6 did not return to the 0 degree unwind position"});
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

    if (!collisionStopped && stage_ == Stage::Recovering && idle)
    {
        stage_ = Stage::Retreating;
    }
    if (stage_ == Stage::Failed)
        autoLoopEnabled_ = false;


}

void PickPlaceMission::ApplyActions(const RobotPanelActions& actions,
    const robotics::RobotState& state,
    robotics::IRobotController& controller,
    robotics::IGripperController& gripper, bool boxGrasped,
    const robotics::CartesianPose& graspBoxPoseInBase,
    const robotics::CartesianPose& placementPoseInBase)
{
    const bool canStart = !taskPaused_ && !boxGrasped &&
        (stage_ == Stage::Ready || stage_ == Stage::Complete || stage_ == Stage::Failed);
    // 첫 임무는 버튼으로 시작하고 성공 뒤에는 Viewer가 상자를 옮기고 Ready로 돌려놓아 자동으로 다음 임무를 시작한다.
    const bool startRequested = canStart && orientationReady_ &&
        (actions.start || (autoLoopEnabled_ && stage_ == Stage::Ready));
    if (startRequested)
    {
        taskPaused_ = false;
        taskSucceeded_ = true;
        placementReleased_ = false;
        missionSucceeded_ = false;
        transitWaypointCount_ = 0;
        autoLoopEnabled_ = true;
        SetStageFromResult(RequestJ6Unwind(state, controller), Stage::UnwindingBeforeTask);
    }
    if (taskPaused_ && actions.resume)
    {
            taskPaused_ = false;
            taskSucceeded_ = true;
            missionSucceeded_ = false;
            transitWaypointCount_ = 0;
            if (boxGrasped)
            {
                placementReleased_ = false;
                if (!state.valid || !state.tcpPoseValid)
                {
                    SetResult({robotics::ErrorCode::InvalidCommand,
                        "RobotPanel: TCP feedback is unavailable for safe resume"});
                    stage_ = Stage::Failed;
                }
                else
                {
                    const glm::dquat tcpOrientation = CaptureAttachedBoxOffset(
                        graspBoxPoseInBase, state, boxOffsetInTool_, boxRotationOffsetInTool_);
                    graspedBoxPoseInBase_ = graspBoxPoseInBase;
                    const double currentBoxHeight = graspedBoxPoseInBase_.positionMeters[1];
                    const double resumeTransitHeight = std::max(currentBoxHeight, kTransitBoxHeightMeters);
                    const double liftDistance = resumeTransitHeight - currentBoxHeight;
                    recoveryPose_ = MakeAttachedBoxTarget(graspedBoxPoseInBase_, liftDistance, tcpOrientation, boxOffsetInTool_);
                    graspedBoxPoseInBase_.positionMeters[1] = resumeTransitHeight - kTransitBoxHeightMeters;
                    SetStageFromResult(MoveTo(recoveryPose_, controller), Stage::RaisingAfterResume);
                }
            }
            else if (placementReleased_)
            {
                SetStageFromResult(RequestJ6Unwind(state, controller), Stage::UnwindingWrist);
            }
            else if (stage_ == Stage::Opening)
            {
                placementReleased_ = true;
                SetStageFromResult(MoveTo(recoveryPose_, controller), Stage::Retreating);
            }
            else
            {
                placementReleased_ = false;
                SetStageFromResult(RequestJ6Unwind(state, controller), Stage::UnwindingBeforeTask);
            }
    }
    else if (actions.stop)
    {
        if (SetResult(controller.Stop()))
        {
            taskPaused_ = true;
            autoLoopEnabled_ = false;
        }
    }

}

PickPlaceMissionSnapshot PickPlaceMission::Snapshot() const
{
    const char* stageName = "Unknown";
    switch (stage_)
    {
    case Stage::Ready: stageName = "Ready"; break;
    case Stage::UnwindingBeforeTask: stageName = "Unwinding J6 before pickup"; break;
    case Stage::MovingAbovePickup: stageName = "Moving above the box"; break;
    case Stage::AligningAbovePickup: stageName = "Aligning over the box"; break;
    case Stage::MovingDownToPickup: stageName = "Lowering to the box"; break;
    case Stage::Closing: stageName = "Closing gripper"; break;
    case Stage::Lifting: stageName = "Lifting the box"; break;
    case Stage::RaisingAfterResume: stageName = "Lifting to resume height"; break;
    case Stage::TransitingToPlacement: stageName = "Planning path to goal"; break;
    case Stage::TransitPathRunning: stageName = "Moving to goal"; break;
    case Stage::MovingToPlacementOverhead: stageName = "Moving above the goal"; break;
    case Stage::AligningAbovePlacement: stageName = "Aligning box over goal"; break;
    case Stage::MovingAbovePlacement: stageName = "Moving to placement height"; break;
    case Stage::MovingDownToPlacement: stageName = "Lowering box"; break;
    case Stage::Opening: stageName = "Opening gripper"; break;
    case Stage::Retreating: stageName = "Retracting from goal"; break;
    case Stage::UnwindingWrist: stageName = "Unwinding J6"; break;
    case Stage::Recovering: stageName = "Retracting after collision"; break;
    case Stage::Complete: stageName = "Complete"; break;
    case Stage::Failed: stageName = "Failed"; break;
    }
    return {stageName, lastResult_.message, taskPaused_, missionSucceeded_, autoLoopEnabled_,
        hasResult_, lastResult_.Ok(), !taskPaused_ &&
            (stage_ == Stage::Ready || stage_ == Stage::Complete || stage_ == Stage::Failed),
        completedMissionCount_};
}

bool PickPlaceMission::ConsumeSuccessEvent() noexcept
{
    const bool result = missionSuccessEventPending_;
    missionSuccessEventPending_ = false;
    return result;
}

void PickPlaceMission::PrepareNextTask() noexcept
{
    stage_ = Stage::Ready;
    taskPaused_ = false;
    missionSucceeded_ = false;
    placementReleased_ = false;
    taskSucceeded_ = true;
    transitWaypointCount_ = 0;
}
}
