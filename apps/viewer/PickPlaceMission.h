#pragma once

#include "robotics/core/IRobotController.h"
#include "robotics/core/IGripperController.h"
#include "robotics/models/RobotSpecification.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace grasplink::viewer
{
struct RobotPanelActions
{
    bool start = false;
    bool resume = false;
    bool stop = false;
};

struct PickPlaceMissionSnapshot
{
    // 단계 이름은 정적 문자열이고, 결과 메시지는 다음 명령 전까지 임무 객체가 보관한다.
    std::string_view stageLabel;
    std::string_view lastMessage;
    bool paused = false;
    bool missionSucceeded = false;
    bool autoRepeat = false;
    bool hasResult = false;
    bool lastRequestAccepted = true;
    bool canStart = false;
    std::uint64_t completedCount = 0;
};

/**
 * @brief 상자 집기와 목표 위치에 놓기까지의 임무 단계를 관리한다.
 * @details ViewerApp은 매 프레임 상태를 먼저 갱신하고, GUI에서 받은 버튼 입력을 그 다음 처리한다.
 * Snapshot의 문자열 보기는 다음 임무 갱신 또는 명령 전까지만 유효하므로 즉시 화면에 전달한다.
 */
class PickPlaceMission final
{
public:
    /** @brief 로봇 모델의 관절 한계를 사용해 임무 객체를 준비한다. */
    explicit PickPlaceMission(const robotics::models::RobotSpecification& specification) noexcept;

    /** @brief 현재 상태를 읽어 임무의 다음 명령과 단계를 갱신한다. */
    void Update(const robotics::RobotState& state, robotics::IRobotController& controller,
        robotics::IGripperController& gripper, bool boxGrasped,
        const robotics::CartesianPose& graspBoxPoseInBase,
        const robotics::CartesianPose& placementPoseInBase);
    /** @brief 패널에서 받은 시작, 재개, 정지 입력을 임무 명령으로 처리한다. */
    void ApplyActions(const RobotPanelActions& actions,
        const robotics::RobotState& state,
        robotics::IRobotController& controller, bool boxGrasped,
        const robotics::CartesianPose& graspBoxPoseInBase);
    /** @brief 화면과 테스트에 필요한 임무 상태를 복사 없이 읽기 전용으로 제공한다. */
    [[nodiscard]] PickPlaceMissionSnapshot Snapshot() const;

    /** @brief 임무 성공 알림이 대기 중이면 한 번 소비하고 대기 상태를 지운다. */
    [[nodiscard]] bool ConsumeSuccessEvent() noexcept;

    /** @brief 성공 후 다음 무작위 상자 작업을 받을 준비를 한다. */
    void PrepareNextTask() noexcept;

private:
    /** @brief 현재 자세를 유지하면서 J6만 0도로 되돌리는 명령을 보낸다. */
    [[nodiscard]] robotics::Result RequestJ6Unwind(const robotics::RobotState& state,
        robotics::IRobotController& controller) const;

    const robotics::models::RobotSpecification& specification_;
    enum class Stage
    {
        Ready, UnwindingBeforeTask, MovingAbovePickup, AligningAbovePickup,
        MovingDownToPickup, Closing, Lifting, MovingAbovePlacement,
        TransitingToPlacement, TransitPathRunning, MovingToPlacementOverhead,
        AligningAbovePlacement, MovingDownToPlacement, Opening, Retreating,
        UnwindingWrist, Recovering, Complete, Failed, RaisingAfterResume
    };

    bool SetResult(robotics::Result result);
    void SetStageFromResult(robotics::Result result, Stage next);

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
    bool taskPaused_ = false;
    std::uint64_t completedMissionCount_ = 0;
};
}
