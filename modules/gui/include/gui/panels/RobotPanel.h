#pragma once

#include "robotics/core/ControlTypes.h"
#include "robotics/core/IGripperController.h"

#include <array>
#include <cstddef>

namespace grasplink::robotics::backends::simulation { class SimRobotController; }

namespace grasplink::gui
{

/**
 * @brief 무작위 위치의 상자를 집어 목표 영역까지 옮기고 놓는 작업을 진행한다.
 * @details 작업은 상자 위 접근, 하강, 손가락 접촉 파지, 들어 올리기, 목표 위 이동, 하강, 해제, 후퇴 순서로 진행한다.
 * TCP는 공구에서 위치와 방향을 지정하는 기준점이며, 상자와 목표 위치는 Robot base 좌표로 전달한다.
 * GLB에 정의된 접근 축을 바닥 방향으로 돌려 손가락이 상자를 위에서 감싸게 한다.
 * 파지한 순간 TCP와 상자 중심 사이의 상대 위치를 저장해 목표 위치에서도 상자 중심이 목표에 오도록 보정한다.
 * 물리 접촉에 따른 정지와 파지 성공 여부는 Viewer의 GripperGraspAdapter가 제공한다.
 * Controller가 바닥 충돌로 안전 관절각에 되돌아오면 패널은 마지막 안전 높이까지 후퇴를 요청한다.
 * 상자와 목표판의 방향을 Robot base 좌표로 받아 목표판 방향에 맞춰 상자를 놓고, 상자 네 꼭짓점이 목표 영역 안에 있을 때 성공 처리한다.
 * Viewer는 성공 알림을 받아 상자와 목표판을 새 무작위 위치와 방향으로 옮기며, 자동 반복 상태가 다음 작업을 시작한다.
 */
class RobotPanel final
{
public:
    /** @brief 작업 단계를 진행하고 현재 상태를 이미 열린 ImGui 창 안에 표시한다. */
    void DrawContents(robotics::backends::simulation::SimRobotController& controller,
        robotics::IGripperController& gripper, bool boxGrasped,
        const robotics::CartesianPose& graspBoxPoseInBase,
        const robotics::CartesianPose& placementPoseInBase);

    /** @brief 이번 프레임에 임무가 성공했는지 나타내는 일회성 알림을 소비한다. */
    [[nodiscard]] bool ConsumeMissionSuccessEvent() noexcept;

    /** @brief 성공한 임무를 마치고 다음 상자 작업을 시작할 준비를 한다. */
    void PrepareNextTask() noexcept;

private:
    enum class Stage
    {
        Ready,
        MovingAbovePickup,
        AligningAbovePickup,
        MovingDownToPickup,
        Closing,
        Lifting,
        MovingAbovePlacement,
        TransitingToPlacement,
        TransitPathRunning,
        MovingToPlacementOverhead,
        AligningAbovePlacement,
        MovingDownToPlacement,
        Opening,
        Retreating,
        UnwindingWrist,
        Recovering,
        Complete,
        Failed
    };

    std::array<double, 4> graspOrientationXyzw_{};
    std::array<double, 4> pickupOrientationXyzw_{};
    std::array<double, 4> placementBoxOrientationXyzw_{};
    robotics::CartesianPose graspedBoxPoseInBase_{};
    std::array<double, 3> boxOffsetInTool_{};
    std::array<double, 4> boxRotationOffsetInTool_{0.0, 0.0, 0.0, 1.0};
    std::array<robotics::CartesianPose, 6> transitWaypoints_{};
    std::size_t transitWaypointCount_ = 0;
    robotics::CartesianPose recoveryPose_{};
    robotics::Result lastResult_{};
    Stage stage_ = Stage::Ready;
    bool orientationReady_ = false;
    bool hasResult_ = false;
    bool taskSucceeded_ = true;
    bool placementReleased_ = false;
    bool missionSucceeded_ = false;
    bool missionSuccessEventPending_ = false;
    bool autoLoopEnabled_ = false;
    std::uint64_t completedMissionCount_ = 0;
};

}
