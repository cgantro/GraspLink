#include "PickPlaceMission.h"
#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/kinematics/RobotInverseKinematics.h"
#include "robotics/models/hanwha/Hcr12a.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>
namespace
{
using namespace grasplink::robotics;

class StubGripper final : public grasplink::robotics::IGripperController
{
public:
    grasplink::robotics::Result Connect() override { return {}; }
    void Disconnect() noexcept override {}
    bool IsConnected() const noexcept override { return true; }
    grasplink::robotics::Result Activate() override { return {}; }
    grasplink::robotics::Result Reset() override { return {}; }
    grasplink::robotics::Result Command(const grasplink::robotics::GripperCommand& command) override
    {
        commands.push_back(command.positionRequest);
        return {};
    }
    grasplink::robotics::Result Stop() override { return {}; }
    grasplink::robotics::GripperState GetState() const override { return state; }
    void Update(double) override {}

    grasplink::robotics::GripperState state{};
    std::vector<std::uint8_t> commands;
};

class FakeRobotController final : public IRobotController
{
public:
    explicit FakeRobotController(
        const models::RobotSpecification& specification = models::hanwha::kHcr12a,
        bool completeMovesImmediately = false)
        : ik_(specification), completeMovesImmediately_(completeMovesImmediately)
    {
        state.jointPositionRadians.assign(specification.jointCount, 0.0);
        state.tcpPose = ik_.EvaluateTcp(state.jointPositionRadians);
        state.mode = RobotMode::Idle;
        state.tcpPoseValid = true;
        state.valid = true;
    }

    Result Connect() override { return {}; }
    void Disconnect() noexcept override {}
    bool IsConnected() const noexcept override { return true; }
    Result MoveJoint(const JointMoveCommand& command) override
    {
        ++jointRequests;
        if (!jointResult)
            return jointResult;
        if (completeMovesImmediately_)
        {
            state.jointPositionRadians = command.targetPositionRadians;
            state.tcpPose = ik_.EvaluateTcp(state.jointPositionRadians);
            state.mode = RobotMode::Idle;
        }
        else
            state.mode = RobotMode::Moving;
        return jointResult;
    }
    Result MoveLinear(const LinearMoveCommand& command) override
    {
        ++linearRequests;
        if (!linearResult)
            return linearResult;
        state.tcpPose = command.targetPose;
        state.mode = completeMovesImmediately_ ? RobotMode::Idle : RobotMode::Moving;
        return linearResult;
    }
    Result MoveLinearPath(const LinearPathMoveCommand& command) override
    {
        ++pathRequests;
        if (!pathResult)
            return pathResult;
        if (completeMovesImmediately_)
        {
            state.tcpPose = command.targetPoses.back();
            state.mode = RobotMode::Idle;
        }
        else
            state.mode = RobotMode::Moving;
        return pathResult;
    }
    Result Stop() override
    {
        ++stopRequests;
        if (stopResult)
            state.mode = RobotMode::Stopped;
        return stopResult;
    }
    RobotState GetState() const override { return state; }
    void Update(double) override {}

    RobotState state;
    Result jointResult{};
    Result linearResult{};
    Result pathResult{};
    Result stopResult{};
    int jointRequests = 0;
    int linearRequests = 0;
    int pathRequests = 0;
    int stopRequests = 0;

private:
    kinematics::DampedLeastSquaresIk ik_;
    bool completeMovesImmediately_ = false;
};

}

TEST(PickPlaceMissionTests, StartPauseAndResumeWithHeldObject)
{
    using namespace grasplink::robotics;
    using grasplink::robotics::backends::simulation::SimRobotController;
    SimRobotController controller(models::hanwha::kHcr12a);
    ASSERT_TRUE(static_cast<bool>(controller.Connect())) << "mission controller connects";
    StubGripper gripper;
    grasplink::viewer::PickPlaceMission mission(models::hanwha::kHcr12a);
    const auto state = controller.GetState();
    CartesianPose box = state.tcpPose;
    CartesianPose goal = state.tcpPose;

    mission.Update(controller.GetStateView(), controller, gripper, false, box, goal);
    mission.ApplyActions({true, false, false}, controller.GetStateView(), controller, false, box);
    ASSERT_EQ(mission.Snapshot().stageLabel, "Unwinding J6 before pickup");

    mission.ApplyActions({false, false, true}, controller.GetStateView(), controller, false, box);
    ASSERT_TRUE(mission.Snapshot().paused);
    mission.ApplyActions({false, true, false}, controller.GetStateView(), controller, true, box);
    const auto resumed = mission.Snapshot();
    ASSERT_FALSE(resumed.paused);
    ASSERT_EQ(resumed.stageLabel, "Lifting to resume height");
}

TEST(PickPlaceMissionTests, UnreachablePickupFailsWithoutMotion)
{
    using namespace grasplink::robotics;
    using grasplink::robotics::backends::simulation::SimRobotController;
    SimRobotController controller(models::hanwha::kHcr12a);
    ASSERT_TRUE(static_cast<bool>(controller.Connect())) << "mission controller connects for failed pickup";
    StubGripper gripper;
    grasplink::viewer::PickPlaceMission mission(models::hanwha::kHcr12a);
    const auto initial = controller.GetState();
    CartesianPose unreachableBox = initial.tcpPose;
    unreachableBox.positionMeters[0] += 5.0;
    const CartesianPose goal = initial.tcpPose;

    mission.Update(controller.GetStateView(), controller, gripper, false, unreachableBox, goal);
    mission.ApplyActions({true, false, false}, controller.GetStateView(), controller, false, unreachableBox);
    ASSERT_EQ(mission.Snapshot().stageLabel, "Unwinding J6 before pickup");

    mission.Update(controller.GetStateView(), controller, gripper, false, unreachableBox, goal);
    const auto failed = mission.Snapshot();
    ASSERT_EQ(failed.stageLabel, "Failed");
    EXPECT_TRUE(failed.hasResult);
    EXPECT_FALSE(failed.lastRequestAccepted);
    EXPECT_FALSE(failed.missionSucceeded);
    EXPECT_EQ(controller.GetStateView().mode, RobotMode::Idle);
}

TEST(PickPlaceMissionTests, RejectedStartCommandFailsImmediately)
{
    FakeRobotController controller;
    StubGripper gripper;
    controller.jointResult = {ErrorCode::Busy, "controller is busy"};
    grasplink::viewer::PickPlaceMission mission(models::hanwha::kHcr12a);

    mission.Update(controller.state, controller, gripper, false, {}, {});
    mission.ApplyActions({true, false, false}, controller.state, controller, false, {});

    const auto snapshot = mission.Snapshot();
    EXPECT_EQ(controller.jointRequests, 1);
    EXPECT_EQ(snapshot.stageLabel, "Failed");
    EXPECT_TRUE(snapshot.hasResult);
    EXPECT_FALSE(snapshot.lastRequestAccepted);
    EXPECT_EQ(snapshot.lastMessage, "controller is busy");
}

TEST(PickPlaceMissionTests, AcceptedCommandWaitsForControllerToBecomeIdle)
{
    FakeRobotController controller;
    StubGripper gripper;
    grasplink::viewer::PickPlaceMission mission(models::hanwha::kHcr12a);

    mission.Update(controller.state, controller, gripper, false, {}, {});
    mission.ApplyActions({true, false, false}, controller.state, controller, false, {});
    mission.Update(controller.state, controller, gripper, false, {}, {});

    EXPECT_EQ(mission.Snapshot().stageLabel, "Unwinding J6 before pickup");
    EXPECT_EQ(controller.jointRequests, 1);
    EXPECT_EQ(controller.linearRequests, 0);
}

TEST(PickPlaceMissionTests, EnvironmentContactRequestsRetreat)
{
    FakeRobotController controller;
    StubGripper gripper;
    grasplink::viewer::PickPlaceMission mission(models::hanwha::kHcr12a);
    CartesianPose box{};
    box.positionMeters = {0.4, 0.2, 0.1};

    mission.Update(controller.state, controller, gripper, false, box, {});
    mission.ApplyActions({true, false, false}, controller.state, controller, false, box);
    controller.state.mode = RobotMode::Idle;
    controller.state.jointPositionRadians.back() = 0.0;
    mission.Update(controller.state, controller, gripper, false, box, {});
    ASSERT_EQ(mission.Snapshot().stageLabel, "Moving above the box");
    controller.state.mode = RobotMode::Idle;
    controller.state.errorCode = ErrorCode::EnvironmentContact;

    mission.Update(controller.state, controller, gripper, false, box, {});

    EXPECT_EQ(mission.Snapshot().stageLabel, "Retracting after collision");
    EXPECT_EQ(controller.linearRequests, 2);
    EXPECT_TRUE(mission.Snapshot().lastRequestAccepted);
}

TEST(PickPlaceMissionTests, UnrelatedControllerErrorDoesNotTriggerCollisionRecovery)
{
    FakeRobotController controller;
    StubGripper gripper;
    grasplink::viewer::PickPlaceMission mission(models::hanwha::kHcr12a);
    CartesianPose box{};
    box.positionMeters = {0.4, 0.2, 0.1};

    mission.Update(controller.state, controller, gripper, false, box, {});
    mission.ApplyActions({true, false, false}, controller.state, controller, false, box);
    controller.state.mode = RobotMode::Idle;
    mission.Update(controller.state, controller, gripper, false, box, {});
    controller.state.mode = RobotMode::Moving;
    controller.state.errorCode = ErrorCode::Fault;

    mission.Update(controller.state, controller, gripper, false, box, {});

    EXPECT_EQ(mission.Snapshot().stageLabel, "Moving above the box");
    EXPECT_EQ(controller.linearRequests, 1);
}

TEST(PickPlaceMissionTests, RejectedStopDoesNotPauseMission)
{
    FakeRobotController controller;
    StubGripper gripper;
    controller.stopResult = {ErrorCode::Fault, "stop rejected"};
    grasplink::viewer::PickPlaceMission mission(models::hanwha::kHcr12a);

    mission.ApplyActions({false, false, true}, controller.state, controller, false, {});

    const auto snapshot = mission.Snapshot();
    EXPECT_EQ(controller.stopRequests, 1);
    EXPECT_FALSE(snapshot.paused);
    EXPECT_EQ(snapshot.stageLabel, "Ready");
    EXPECT_FALSE(snapshot.lastRequestAccepted);
    EXPECT_EQ(snapshot.lastMessage, "stop rejected");
}

TEST(PickPlaceMissionTests, ResumeWithHeldObjectRequiresValidTcpFeedback)
{
    FakeRobotController controller;
    StubGripper gripper;
    grasplink::viewer::PickPlaceMission mission(models::hanwha::kHcr12a);
    mission.ApplyActions({false, false, true}, controller.state, controller, false, {});
    ASSERT_TRUE(mission.Snapshot().paused);
    controller.state.valid = false;
    controller.state.tcpPoseValid = false;

    mission.ApplyActions({false, true, false}, controller.state, controller, true, {});

    const auto snapshot = mission.Snapshot();
    EXPECT_FALSE(snapshot.paused);
    EXPECT_EQ(snapshot.stageLabel, "Failed");
    EXPECT_FALSE(snapshot.lastRequestAccepted);
    EXPECT_EQ(snapshot.lastMessage, "RobotPanel: TCP feedback is unavailable for safe resume");
    EXPECT_EQ(controller.jointRequests, 0);
    EXPECT_EQ(controller.linearRequests, 0);
}

TEST(PickPlaceMissionTests, SuccessfulCyclePublishesOneCompletionEvent)
{
    FakeRobotController controller(models::hanwha::kHcr12a, true);
    StubGripper gripper;
    grasplink::viewer::PickPlaceMission mission(models::hanwha::kHcr12a);
    CartesianPose box{};
    box.positionMeters = controller.state.tcpPose.positionMeters;
    box.positionMeters[1] -= 0.25;
    CartesianPose placement = box;

    mission.Update(controller.state, controller, gripper, false, box, placement);
    mission.ApplyActions({true, false, false}, controller.state, controller, false, box);
    bool boxGrasped = false;
    for (int tick = 0; tick < 40 && mission.Snapshot().stageLabel != "Complete" &&
        mission.Snapshot().stageLabel != "Failed"; ++tick)
    {
        if (mission.Snapshot().stageLabel == "Closing gripper")
            boxGrasped = true;
        if (mission.Snapshot().stageLabel == "Opening gripper")
            boxGrasped = false;
        const auto stageBeforeUpdate = mission.Snapshot().stageLabel;
        mission.Update(controller.state, controller, gripper, boxGrasped, box, placement);
        if (mission.Snapshot().stageLabel == "Failed")
            ADD_FAILURE() << "Mission failed while processing " << stageBeforeUpdate
                << ": " << mission.Snapshot().lastMessage;
    }

    const auto snapshot = mission.Snapshot();
    ASSERT_EQ(snapshot.stageLabel, "Complete") << snapshot.lastMessage;
    EXPECT_TRUE(snapshot.missionSucceeded);
    EXPECT_EQ(snapshot.completedCount, 1U);
    EXPECT_TRUE(mission.ConsumeSuccessEvent());
    EXPECT_FALSE(mission.ConsumeSuccessEvent());
}

TEST(PickPlaceMissionTests, GripperMissReleasesAndFailsWithoutSuccessEvent)
{
    FakeRobotController controller(models::hanwha::kHcr12a, true);
    StubGripper gripper;
    gripper.state.mode = GripperMode::Idle;
    gripper.state.objectStatus = GripperObjectStatus::AtRequestedPosition;
    grasplink::viewer::PickPlaceMission mission(models::hanwha::kHcr12a);
    CartesianPose box{};
    box.positionMeters = controller.state.tcpPose.positionMeters;
    box.positionMeters[1] -= 0.25;

    mission.Update(controller.state, controller, gripper, false, box, box);
    mission.ApplyActions({true, false, false}, controller.state, controller, false, box);
    for (int tick = 0; tick < 40 && mission.Snapshot().stageLabel != "Complete" &&
        mission.Snapshot().stageLabel != "Failed"; ++tick)
        mission.Update(controller.state, controller, gripper, false, box, box);

    const auto snapshot = mission.Snapshot();
    EXPECT_EQ(snapshot.stageLabel, "Failed");
    EXPECT_FALSE(snapshot.missionSucceeded);
    EXPECT_EQ(snapshot.completedCount, 0U);
    EXPECT_EQ(gripper.commands, (std::vector<std::uint8_t>{255, 0}));
    EXPECT_FALSE(mission.ConsumeSuccessEvent());
}

TEST(PickPlaceMissionTests, RejectedTransitPathPreservesControllerError)
{
    FakeRobotController controller(models::hanwha::kHcr12a, true);
    controller.pathResult = {ErrorCode::Unsupported, "path execution unavailable"};
    StubGripper gripper;
    grasplink::viewer::PickPlaceMission mission(models::hanwha::kHcr12a);
    CartesianPose box{};
    box.positionMeters = controller.state.tcpPose.positionMeters;
    box.positionMeters[1] -= 0.25;

    mission.Update(controller.state, controller, gripper, false, box, box);
    mission.ApplyActions({true, false, false}, controller.state, controller, false, box);
    bool boxGrasped = false;
    for (int tick = 0; tick < 40 && mission.Snapshot().stageLabel != "Complete" &&
        mission.Snapshot().stageLabel != "Failed"; ++tick)
    {
        boxGrasped = mission.Snapshot().stageLabel == "Closing gripper" || boxGrasped;
        mission.Update(controller.state, controller, gripper, boxGrasped, box, box);
    }

    const auto snapshot = mission.Snapshot();
    EXPECT_EQ(snapshot.stageLabel, "Failed");
    EXPECT_EQ(snapshot.lastMessage, "path execution unavailable");
    EXPECT_EQ(controller.pathRequests, 1);
    EXPECT_FALSE(snapshot.missionSucceeded);
}
