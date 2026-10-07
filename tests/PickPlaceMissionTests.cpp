#include "PickPlaceMission.h"
#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/models/hanwha/Hcr12a.h"
#include "TestSupport.h"

#include <iostream>

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
    Require(static_cast<bool>(controller.Connect()), "mission controller connects");
    StubGripper gripper;
    Require(static_cast<bool>(gripper.Connect()), "stub gripper connects");
    grasplink::viewer::PickPlaceMission mission(models::hanwha::kHcr12a);
    const auto state = controller.GetState();
    CartesianPose box = state.tcpPose;
    CartesianPose goal = state.tcpPose;

    mission.Update(controller.GetStateView(), controller, gripper, false, box, goal);
    mission.ApplyActions({true, false, false}, controller.GetStateView(), controller,
        gripper, false, box, goal);
    Require(mission.Snapshot().stageLabel == "Unwinding J6 before pickup",
        "Start issues the J6 unwind and exposes its stage");

    mission.ApplyActions({false, false, true}, controller.GetStateView(), controller,
        gripper, false, box, goal);
    Require(mission.Snapshot().paused, "Stop pauses the mission without clearing its stage");
    mission.ApplyActions({false, true, false}, controller.GetStateView(), controller,
        gripper, true, box, goal);
    const auto resumed = mission.Snapshot();
    Require(!resumed.paused, "Resume clears the paused state");
    Require(resumed.stageLabel == "Lifting to resume height",
        "Resume with a held object commands a safe lift before transit planning");
}
}

int main()
{
    try
    {
        CheckStartAndPauseResumeWithHeldObject();
        std::cout << "Pick and place mission checks passed\n";
        return 0;
    }
    catch (const std::exception& exception)
    {
        std::cerr << exception.what() << '\n';
        return 1;
    }
}
