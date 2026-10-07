#include "PickPlaceMission.h"
#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/models/hanwha/Hcr12a.h"

#include <gtest/gtest.h>

namespace
{
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
    mission.ApplyActions({true, false, false}, controller.GetStateView(), controller,
        gripper, false, box, goal);
    ASSERT_EQ(mission.Snapshot().stageLabel, "Unwinding J6 before pickup");

    mission.ApplyActions({false, false, true}, controller.GetStateView(), controller,
        gripper, false, box, goal);
    ASSERT_TRUE(mission.Snapshot().paused);
    mission.ApplyActions({false, true, false}, controller.GetStateView(), controller,
        gripper, true, box, goal);
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
    mission.ApplyActions({true, false, false}, controller.GetStateView(), controller,
        gripper, false, unreachableBox, goal);
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
