#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/models/hanwha/Hcr12a.h"
#include "robotics/runtime/FixedControlLoop.h"
#include "TestSupport.h"

#include <iostream>
#include <limits>

using namespace grasplink::robotics;

int main()
{
    try
    {
        runtime::FixedControlLoop loop(0.004, 0.1);
        std::size_t ticks = 0;
        auto tick = [&](double dt) { RequireNear(dt, 0.004, 1e-12, "fixed dt"); ++ticks; };
        Require(loop.Advance(0.010, tick) == 2, "two ticks");
        RequireNear(loop.GetAccumulatorSeconds(), 0.002, 1e-12, "remaining time");
        Require(loop.Advance(0.002, tick) == 1, "carry-over tick");
        loop.Reset();
        loop.Advance(3.0, tick);
        Require(ticks <= 28 && ticks >= 27, "clamp discards excess elapsed time");
        Require(loop.GetInterpolationAlpha() >= 0.0 && loop.GetInterpolationAlpha() < 1.0, "interpolation remainder");
        const auto before = ticks;
        loop.Advance(std::numeric_limits<double>::quiet_NaN(), tick);
        loop.Advance(-1.0, tick);
        Require(ticks == before, "invalid elapsed time");

        backends::simulation::SimRobotController controller(models::hanwha::kHcr12a);
        Require(static_cast<bool>(controller.Connect()), "connect");
        JointMoveCommand command;
        command.targetPositionRadians.assign(6, 0.0);
        command.targetPositionRadians[0] = 0.5;
        Require(static_cast<bool>(controller.MoveJoint(command)), "move");
        controller.Update(0.004);
        RequireNear(controller.GetState().jointPositionRadians[0], 2.268928 * 0.004, 1e-12, "model velocity limit");
        const auto unchanged = controller.GetState().jointPositionRadians;
        command.targetPositionRadians[0] = 10.0;
        Require(controller.MoveJoint(command).code == ErrorCode::InvalidCommand, "out-of-range command");
        Require(controller.GetState().jointPositionRadians == unchanged, "invalid command preserves state");
        command.targetPositionRadians[0] = -0.5;
        Require(static_cast<bool>(controller.MoveJoint(command)), "re-target while moving");
        controller.Update(0.004);
        Require(controller.GetState().jointPositionRadians[0] < unchanged[0], "new target replaces old target");
        Require(static_cast<bool>(controller.Stop()), "stop");
        const auto stopped = controller.GetState().jointPositionRadians;
        controller.Update(1.0);
        Require(controller.GetState().jointPositionRadians == stopped, "stop freezes q");
        Require(!controller.GetState().tcpPoseValid, "controller has no synthetic TCP feedback");
        Require(controller.MoveLinear({}).code == ErrorCode::Unsupported, "no IK trajectory");
        controller.Disconnect();
        Require(!controller.GetState().valid, "disconnected state invalid");

        auto joint = models::hanwha::kHcr12aJoints.front();
        auto specification = models::hanwha::kHcr12a;
        specification.joints = &joint;
        specification.jointCount = 1;
        joint.maxVelocityRadiansPerSecond = -1.0;
        ExpectThrows<std::invalid_argument>([&] { backends::simulation::SimRobotController bad(specification); }, "invalid model velocity");
        joint.maxVelocityRadiansPerSecond = 1.0;
        joint.minPositionRadians = 0.1;
        ExpectThrows<std::invalid_argument>([&] { backends::simulation::SimRobotController bad(specification); }, "model zero pose outside limits");
        std::cout << "Controller and fixed-loop checks passed\n";
        return 0;
    }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
