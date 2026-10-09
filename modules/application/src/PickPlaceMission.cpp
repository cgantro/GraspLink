#include "application/PickPlaceMission.h"
#include "application/PickPlaceConfig.h"
#include <glm/gtc/constants.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <string_view>
#include <utility>

namespace grasplink::application
{
namespace
{
namespace config = pick_place::config;
// 실제 Gripper convex hull은 TCP가 상자 중심에서 10 mm 위일 때 바닥과 겹쳤다. 20 mm에서 간격이 생겨 여유 5 mm를 더 둔다.
// 작업공간 상한은 안전 여유값으로 두고, 각 경로 구간의 실제 속도는 IK가 만든 관절 변화량과 모델 관절 속도로 정한다.

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
    const double boxHalfSide = config::boxSideMeters * 0.5;
    const double targetHalfSide = config::placementAreaSideMeters * 0.5;
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

const char* RuntimeFaultReason(robotics::ErrorCode errorCode)
{
    switch (errorCode)
    {
    case robotics::ErrorCode::IkDidNotConverge: return "runtime IK did not converge";
    case robotics::ErrorCode::JointLimitReached: return "a joint reached its limit";
    case robotics::ErrorCode::EnvironmentContact: return "the robot contacted the environment";
    case robotics::ErrorCode::SelfCollision: return "the robot self-collided";
    case robotics::ErrorCode::AttachedObjectCollision: return "the attached object collided";
    default: return "the robot controller reported a fault";
    }
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
    const std::array<double, 4>& targetBoxOrientation, const std::array<double, 4>& boxRotationOffsetInTool,
    const std::array<double, 3>& boxOffsetInTool)
{
    const glm::dquat boxRotationOffset = Orientation(boxRotationOffsetInTool);
    const glm::dquat tcpOrientation = glm::normalize(
        Orientation(targetBoxOrientation) * glm::inverse(boxRotationOffset));
    const glm::dvec3 boxOffset = tcpOrientation *
        glm::dvec3{boxOffsetInTool[0], boxOffsetInTool[1], boxOffsetInTool[2]};
    robotics::CartesianPose target{};
    target.orientationXyzw = QuaternionXyzw(tcpOrientation);
    target.positionMeters = {boxPose.positionMeters[0] - boxOffset.x,
        boxPose.positionMeters[1] + heightOffset - boxOffset.y,
        boxPose.positionMeters[2] - boxOffset.z};
    return target;
}

robotics::Result MovePoseTo(const robotics::CartesianPose& pose, robotics::IRobotController& controller)
{
    return controller.BeginPosePlanning(pose);
}

robotics::LinearPathMoveCommand LinearCommandTo(const robotics::CartesianPose& pose)
{
    robotics::LinearPathMoveCommand command;
    command.targetPoses.push_back(pose);
    command.maxLinearVelocityMetersPerSecond = config::linearVelocityMetersPerSecond;
    command.maxAngularVelocityRadiansPerSecond = config::angularVelocityRadiansPerSecond;
    command.maxLinearAccelerationMetersPerSecondSquared = config::linearAccelerationMetersPerSecondSquared;
    command.maxAngularAccelerationRadiansPerSecondSquared = config::angularAccelerationRadiansPerSecondSquared;
    return command;
}

robotics::Result MoveLinearTo(const robotics::CartesianPose& pose, robotics::IRobotController& controller)
{
    const auto command = LinearCommandTo(pose);
    return controller.BeginLinearPathPlanning(command);
}

robotics::Result CommandGripper(std::uint8_t position, robotics::IGripperController& gripper)
{
    robotics::GripperCommand command;
    command.positionRequest = position;
    command.speedRequest = 255;
    command.forceRequest = 128;
    return gripper.Command(command);
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

void PickPlaceMission::SetStageFromMotionResult(
    robotics::Result result, Stage next, robotics::IRobotController& controller)
{
    if (!SetResult(std::move(result)))
    {
        stage_ = Stage::Failed;
        return;
    }
    if (controller.IsMotionPlanning())
    {
        pendingStageAfterPlanning_ = next;
        stage_ = Stage::PlanningMotion;
        return;
    }
    stage_ = next;
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
    const bool activeMission = stage_ != Stage::Ready && stage_ != Stage::Complete && stage_ != Stage::Failed;
    if (!taskPaused_ && activeMission && state.mode == robotics::RobotMode::Fault)
    {
        const auto errorCode = state.errorCode == robotics::ErrorCode::None
            ? robotics::ErrorCode::Fault : state.errorCode;
        SetResult({errorCode, std::string{"RobotPanel: motion fault: "} + RuntimeFaultReason(errorCode)});
        taskSucceeded_ = false;
        stage_ = Stage::Failed;
        autoLoopEnabled_ = false;
        return;
    }
    if (stage_ == Stage::PlanningMotion && !taskPaused_)
    {
        if (controller.IsMotionPlanning())
            return;
        auto result = controller.TakeMotionPlanningResult();
        if (!result)
        {
            SetResult({robotics::ErrorCode::Fault, "RobotPanel: linear path planner ended without a result"});
            stage_ = Stage::Failed;
            return;
        }
        SetStageFromResult(std::move(*result), pendingStageAfterPlanning_);
        pendingStageAfterPlanning_ = Stage::Ready;
        return;
    }
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
        SetStageFromMotionResult(MoveLinearTo(recoveryPose_, controller), Stage::Recovering, controller);
    }
    else
    {
        switch (stage_)
        {
        case Stage::UnwindingBeforeTask:
            if (idle)
            {
                if (state.jointPositionRadians.empty() ||
                    std::abs(state.jointPositionRadians.back()) > config::j6UnwindToleranceRadians)
                {
                    SetResult({robotics::ErrorCode::Fault,
                        "RobotPanel: J6 did not return to the 0 degree unwind position"});
                    stage_ = Stage::Failed;
                }
                else
                {
                    const double boxYawRadians = glm::radians(YawDegrees(graspBoxPoseInBase.orientationXyzw));
                    const glm::dquat desiredPickupOrientation = glm::normalize(
                        glm::angleAxis(boxYawRadians, glm::dvec3{0.0, 1.0, 0.0}) * Orientation(graspOrientationXyzw_));
                    pickupOrientationXyzw_ = QuaternionXyzw(desiredPickupOrientation);
                    const auto approach = MakeBoxTarget(graspBoxPoseInBase, config::approachHeightMeters,
                        pickupOrientationXyzw_);
                    const auto descent = MakeBoxTarget(graspBoxPoseInBase, config::graspClearanceMeters,
                        pickupOrientationXyzw_);
                    recoveryPose_ = approach;
                    SetStageFromMotionResult(controller.BeginPosePlanningWithLinearContinuation(
                        approach, LinearCommandTo(descent)), Stage::MovingAbovePickup, controller);
                }
            }
            break;
        case Stage::MovingAbovePickup:
            if (idle)
            {
                const auto descent = MakeBoxTarget(graspBoxPoseInBase, config::graspClearanceMeters,
                    pickupOrientationXyzw_);
                SetStageFromMotionResult(MoveLinearTo(descent, controller), Stage::MovingDownToPickup, controller);
            }
            break;
        case Stage::MovingDownToPickup:
            if (idle)
                SetStageFromResult(CommandGripper(255, gripper), Stage::Closing);
            break;
        case Stage::Closing:
            if (!taskPaused_ && boxGrasped && state.tcpPoseValid)
            {
                // Constraint는 파지 순간의 물체와 그리퍼 상대 자세를 유지한다. 이 차이를 TCP의 Local 좌표로 저장하면 방향이 바뀌어도 목표 위치에 상자 중심을 맞출 수 있다.
                CaptureAttachedBoxOffset(graspBoxPoseInBase, state, boxOffsetInTool_, boxRotationOffsetInTool_);
                graspedBoxPoseInBase_ = graspBoxPoseInBase;
                recoveryPose_ = MakeAttachedBoxTarget(graspBoxPoseInBase, config::approachHeightMeters,
                    graspBoxPoseInBase.orientationXyzw, boxRotationOffsetInTool_, boxOffsetInTool_);
                SetStageFromMotionResult(MoveLinearTo(recoveryPose_, controller), Stage::Lifting, controller);
            }
            else if (gripperCloseFinished)
            {
                taskSucceeded_ = false;
                if (!SetResult(CommandGripper(0, gripper)))
                    stage_ = Stage::Failed;
                else
                    SetStageFromMotionResult(MoveLinearTo(recoveryPose_, controller), Stage::Recovering, controller);
            }
            break;
        case Stage::Lifting:
            if (idle)
            {
                recoveryPose_ = MakeAttachedBoxTarget(graspedBoxPoseInBase_, config::transitBoxHeightMeters,
                    graspedBoxPoseInBase_.orientationXyzw, boxRotationOffsetInTool_, boxOffsetInTool_);
                SetStageFromMotionResult(MoveLinearTo(recoveryPose_, controller), Stage::TransitingToPlacement, controller);
            }
            break;
        case Stage::RaisingAfterResume:
            if (idle)
                stage_ = Stage::TransitingToPlacement;
            break;
        case Stage::TransitingToPlacement:
            if (idle)
            {
                const glm::dquat boxRotationOffset = Orientation(boxRotationOffsetInTool_);
                const glm::dquat currentTcp = TcpOrientation(state);
                const glm::dquat chosenTcp = ClosestSquarePlacementOrientation(
                    Orientation(placementPoseInBase.orientationXyzw), boxRotationOffset, currentTcp);
                placementBoxOrientationXyzw_ = QuaternionXyzw(chosenTcp * boxRotationOffset);
                recoveryPose_ = MakeAttachedBoxTarget(placementPoseInBase, config::transitBoxHeightMeters,
                    QuaternionXyzw(currentTcp * boxRotationOffset), boxRotationOffsetInTool_, boxOffsetInTool_);
                SetStageFromMotionResult(MovePoseTo(recoveryPose_, controller), Stage::MovingToPlacementOverhead, controller);
            }
            break;
        case Stage::MovingToPlacementOverhead:
            if (idle)
            {
                recoveryPose_ = MakeAttachedBoxTarget(placementPoseInBase, config::approachHeightMeters,
                    placementBoxOrientationXyzw_, boxRotationOffsetInTool_, boxOffsetInTool_);
                const auto placement = MakeAttachedBoxTarget(placementPoseInBase, 0.0,
                    placementBoxOrientationXyzw_, boxRotationOffsetInTool_, boxOffsetInTool_);
                SetStageFromMotionResult(controller.BeginPosePlanningWithLinearContinuation(
                    recoveryPose_, LinearCommandTo(placement)), Stage::AligningAbovePlacement, controller);
            }
            break;
        case Stage::AligningAbovePlacement:
            if (idle)
            {
                const auto placement = MakeAttachedBoxTarget(placementPoseInBase, 0.0,
                    placementBoxOrientationXyzw_, boxRotationOffsetInTool_, boxOffsetInTool_);
                SetStageFromMotionResult(MoveLinearTo(placement, controller), Stage::MovingDownToPlacement, controller);
            }
            break;
        case Stage::MovingDownToPlacement:
            if (idle)
                SetStageFromResult(CommandGripper(0, gripper), Stage::Opening);
            break;
        case Stage::Opening:
            if (!taskPaused_ && !boxGrasped)
            {
                placementReleased_ = true;
                SetStageFromMotionResult(MoveLinearTo(recoveryPose_, controller), Stage::Retreating, controller);
            }
            break;
        case Stage::Retreating:
            if (idle)
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
            break;
        case Stage::UnwindingWrist:
            if (idle)
            {
                const bool wristUnwound = !state.jointPositionRadians.empty() &&
                    std::abs(state.jointPositionRadians.back()) <= config::j6UnwindToleranceRadians;
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
            break;
        case Stage::Ready:
        case Stage::PlanningMotion:
        case Stage::Recovering:
        case Stage::Complete:
        case Stage::Failed:
            break;
        }
    }

    if (!collisionStopped && stage_ == Stage::Recovering && idle)
    {
        stage_ = Stage::Retreating;
    }
    if (stage_ == Stage::Failed)
        autoLoopEnabled_ = false;


}

void PickPlaceMission::ApplyActions(const PickPlaceMissionActions& actions,
    const robotics::RobotState& state, robotics::IRobotController& controller, bool boxGrasped,
    const robotics::CartesianPose& graspBoxPoseInBase)
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
        autoLoopEnabled_ = true;
        SetStageFromResult(RequestJ6Unwind(state, controller), Stage::UnwindingBeforeTask);
    }
    if (taskPaused_ && actions.resume)
    {
            taskPaused_ = false;
            taskSucceeded_ = true;
            missionSucceeded_ = false;
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
                    const double resumeTransitHeight = std::max(currentBoxHeight, config::transitBoxHeightMeters);
                    const double liftDistance = resumeTransitHeight - currentBoxHeight;
                    recoveryPose_ = MakeAttachedBoxTarget(graspedBoxPoseInBase_, liftDistance,
                        graspedBoxPoseInBase_.orientationXyzw, boxRotationOffsetInTool_, boxOffsetInTool_);
                    graspedBoxPoseInBase_.positionMeters[1] = resumeTransitHeight - config::transitBoxHeightMeters;
                    SetStageFromMotionResult(MoveLinearTo(recoveryPose_, controller), Stage::RaisingAfterResume, controller);
                }
            }
            else if (placementReleased_)
            {
                SetStageFromResult(RequestJ6Unwind(state, controller), Stage::UnwindingWrist);
            }
            else if (stage_ == Stage::Opening)
            {
                placementReleased_ = true;
                SetStageFromMotionResult(MoveLinearTo(recoveryPose_, controller), Stage::Retreating, controller);
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
            if (stage_ == Stage::PlanningMotion)
                controller.TakeMotionPlanningResult();
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
    case Stage::MovingDownToPickup: stageName = "Lowering to the box"; break;
    case Stage::Closing: stageName = "Closing gripper"; break;
    case Stage::Lifting: stageName = "Lifting the box"; break;
    case Stage::RaisingAfterResume: stageName = "Lifting to resume height"; break;
    case Stage::TransitingToPlacement: stageName = "Planning path to goal"; break;
    case Stage::PlanningMotion: stageName = "Planning motion"; break;
    case Stage::MovingToPlacementOverhead: stageName = "Moving above the goal"; break;
    case Stage::AligningAbovePlacement: stageName = "Aligning box over goal"; break;
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
}
}
