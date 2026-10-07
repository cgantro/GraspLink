#include "PickPlaceMission.h"
#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/models/hanwha/Hcr12a.h"

#include <gtest/gtest.h>

namespace
{
using namespace grasplink::robotics;

class StubGripper final : public grasplink::robotics::IGripperController
{
public:
    grasplink::robotics::Result Connect() override { connected_ = true; return {}; }
    void Disconnect() noexcept override { connected_ = false; }
    bool IsConnected() const noexcept override { return connected_; }
    grasplink::robotics::Result Activate() override { return {}; }
    grasplink::robotics::Result Reset() override { return {}; }
    grasplink::robotics::Result Command(const grasplink::robotics::GripperCommand&) override { return {}; }
    grasplink::robotics::Result Stop() override { return {}; }
    grasplink::robotics::GripperState GetState() const override { return {}; }
    void Update(double) override {}

private:
    bool connected_ = false;
};

class ScriptedRobotController final : public IRobotController
{
public:
    ScriptedRobotController()
    {
        state.jointPositionRadians = JointVector(6, 0.0);
        state.mode = RobotMode::Idle;
        state.tcpPoseValid = true;
        state.valid = true;
    }

    Result Connect() override { connected = true; return {}; }
    void Disconnect() noexcept override { connected = false; }
    bool IsConnected() const noexcept override { return connected; }
    Result MoveJoint(const JointMoveCommand& command) override
    {
        ++jointRequests;
        lastJointCommand = command;
        if (jointResult)
            state.mode = RobotMode::Moving;
        return jointResult;
    }
    Result MoveLinear(const LinearMoveCommand& command) override
    {
        ++linearRequests;
        lastLinearCommand = command;
        if (linearResult)
            state.mode = RobotMode::Moving;
        return linearResult;
    }
    Result MoveLinearPath(const LinearPathMoveCommand& command) override
    {
        ++pathRequests;
        lastPathCommand = command;
        if (pathResult)
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
    JointMoveCommand lastJointCommand{};
    LinearMoveCommand lastLinearCommand{};
    LinearPathMoveCommand lastPathCommand{};
    int jointRequests = 0;
    int linearRequests = 0;
    int pathRequests = 0;
    int stopRequests = 0;

private:
    bool connected = false;
};

void CheckStartAndPauseResumeWithHeldObject()
{
    using namespace grasplink::robotics;
    using grasplink::robotics::backends::simulation::SimRobotController;
    SimRobotController controller(models::hanwha::kHcr12a);
    ASSERT_TRUE(static_cast<bool>(controller.Connect())) << "mission controller connects";
    StubGripper gripper;
    ASSERT_TRUE(static_cast<bool>(gripper.Connect())) << "stub gripper connects";
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

void CheckUnreachablePickupFailsWithoutMotion()
{
    using namespace grasplink::robotics;
    using grasplink::robotics::backends::simulation::SimRobotController;
    SimRobotController controller(models::hanwha::kHcr12a);
    ASSERT_TRUE(static_cast<bool>(controller.Connect())) << "mission controller connects for failed pickup";
    StubGripper gripper;
    ASSERT_TRUE(static_cast<bool>(gripper.Connect())) << "stub gripper connects for failed pickup";
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
}

TEST(PickPlaceMissionTests, StartPauseAndResumeWithHeldObject)
{
    CheckStartAndPauseResumeWithHeldObject();
}

TEST(PickPlaceMissionTests, UnreachablePickupFailsWithoutMotion)
{
    CheckUnreachablePickupFailsWithoutMotion();
}

TEST(PickPlaceMissionTests, RejectedStartCommandFailsImmediately)
{
    ScriptedRobotController controller;
    StubGripper gripper;
    ASSERT_TRUE(gripper.Connect());
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
    ScriptedRobotController controller;
    StubGripper gripper;
    ASSERT_TRUE(gripper.Connect());
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
    ScriptedRobotController controller;
    StubGripper gripper;
    ASSERT_TRUE(gripper.Connect());
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
    ScriptedRobotController controller;
    StubGripper gripper;
    ASSERT_TRUE(gripper.Connect());
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
    ScriptedRobotController controller;
    StubGripper gripper;
    ASSERT_TRUE(gripper.Connect());
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
    ScriptedRobotController controller;
    StubGripper gripper;
    ASSERT_TRUE(gripper.Connect());
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
