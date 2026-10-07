#include "gui/panels/detail/RobotPanelMotion.h"

#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/models/hanwha/Hcr12a.h"
#include "TestSupport.h"

#include <glm/gtc/constants.hpp>

#include <array>
#include <cmath>
#include <iostream>

using namespace grasplink::robotics;
using grasplink::gui::detail::PlanWristOnlyTarget;
using grasplink::robotics::backends::simulation::SimRobotController;

namespace
{
void CheckWristTargetChangesOnlyJ6()
{
    SimRobotController controller(models::hanwha::kHcr12a);
    Require(static_cast<bool>(controller.Connect()), "wrist planning controller connects");
    JointVector downFacingJoints(6, 0.0);
    downFacingJoints[4] = glm::half_pi<double>();
    Require(static_cast<bool>(controller.MoveJoint({downFacingJoints, 1.0, 1.0})),
        "the wrist first reaches the down-facing grasp orientation");
    for (int tick = 0; tick < 1000 && controller.GetState().mode == RobotMode::Moving; ++tick)
        controller.Update(0.01);
    Require(controller.GetState().mode == RobotMode::Idle, "the down-facing wrist orientation is reached");
    const RobotState state = controller.GetState();
    const glm::dquat current{
        state.tcpPose.orientationXyzw[3], state.tcpPose.orientationXyzw[0],
        state.tcpPose.orientationXyzw[1], state.tcpPose.orientationXyzw[2]};
    const glm::dvec3 j6Axis = glm::normalize(glm::dvec3{0.0, -1.0, 0.0});
    const glm::dquat target = glm::angleAxis(0.35, j6Axis) * current;
    const std::array<double, 4> targetXyzw{target.x, target.y, target.z, target.w};

    const auto plan = PlanWristOnlyTarget(models::hanwha::kHcr12a, state, targetXyzw);
    Require(plan.has_value(), "an on-axis TCP accepts a pure J6 rotation");
    Require(plan->jointPositionRadians.size() == state.jointPositionRadians.size(), "wrist plan preserves joint count");
    for (std::size_t joint = 0; joint + 1 < state.jointPositionRadians.size(); ++joint)
        RequireNear(plan->jointPositionRadians[joint], state.jointPositionRadians[joint], 1e-12,
            "wrist alignment does not change J1 through J5");
    RequireNear(plan->jointPositionRadians.back(), 0.35, 1e-10, "wrist alignment changes J6 by the requested amount");

    const std::array<double, 3> axialBoxOffset{0.0, 0.0, 0.025};
    Require(PlanWristOnlyTarget(models::hanwha::kHcr12a, state, targetXyzw, axialBoxOffset).has_value(),
        "an attached box centered on the J6 axis stays in place during wrist alignment");
    RobotState measuredToolOffset = state;
    measuredToolOffset.tcpPose.positionMeters[0] += 0.00144;
    Require(PlanWristOnlyTarget(models::hanwha::kHcr12a, measuredToolOffset, targetXyzw, axialBoxOffset).has_value(),
        "the measured 1.44 mm fingertip-center offset stays within the bounded wrist sweep");
    const std::array<double, 3> radialBoxOffset{0.02, 0.0, 0.0};
    const auto radialBoxPlan = PlanWristOnlyTarget(models::hanwha::kHcr12a, state, targetXyzw, radialBoxOffset);
    Require(radialBoxPlan.has_value(), "a small attached-box arc does not block wrist alignment");
    Require(radialBoxPlan->attachedBoxSweepMeters < 0.02,
        "the wrist plan reports the attached box center movement during rotation");
    const std::array<double, 3> largeRadialBoxOffset{0.10, 0.0, 0.0};
    Require(!PlanWristOnlyTarget(models::hanwha::kHcr12a, state, targetXyzw, largeRadialBoxOffset).has_value(),
        "a wrist rotation is rejected when the attached box sweep would exceed two centimeters");

    RobotState offAxisTcp = state;
    offAxisTcp.tcpPose.positionMeters[0] += 0.01;
    Require(!PlanWristOnlyTarget(models::hanwha::kHcr12a, offAxisTcp, targetXyzw).has_value(),
        "wrist alignment is rejected when the actual TCP is offset from the J6 axis");

    const glm::dquat otherAxisTarget = glm::angleAxis(0.2, glm::dvec3{1.0, 0.0, 0.0}) * current;
    Require(!PlanWristOnlyTarget(models::hanwha::kHcr12a, state,
        {otherAxisTarget.x, otherAxisTarget.y, otherAxisTarget.z, otherAxisTarget.w}).has_value(),
        "a requested tilt is rejected because J6 cannot produce it alone");
}

void CheckWristTargetRespectsJ6Limits()
{
    SimRobotController controller(models::hanwha::kHcr12a);
    Require(static_cast<bool>(controller.Connect()), "limit fixture controller connects");
    JointVector start(6, 0.0);
    start.back() = glm::radians(350.0);
    Require(static_cast<bool>(controller.MoveJoint({start, 1.0, 1.0})), "J6 moves to 350 degrees");
    for (int tick = 0; tick < 1000 && controller.GetState().mode == RobotMode::Moving; ++tick)
        controller.Update(0.01);
    Require(controller.GetState().mode == RobotMode::Idle, "J6 reaches 350 degrees");

    const RobotState state = controller.GetState();
    const glm::dquat current{
        state.tcpPose.orientationXyzw[3], state.tcpPose.orientationXyzw[0],
        state.tcpPose.orientationXyzw[1], state.tcpPose.orientationXyzw[2]};
    const glm::dquat target = glm::angleAxis(glm::radians(20.0), glm::dvec3{0.0, 0.0, 1.0}) * current;
    const auto plan = PlanWristOnlyTarget(models::hanwha::kHcr12a, state,
        {target.x, target.y, target.z, target.w});
    Require(plan.has_value(), "an equivalent J6 target inside the official range remains available");
    RequireNear(plan->jointPositionRadians.back(), glm::radians(10.0), 1e-9,
        "J6 target is represented at 10 degrees rather than exceeding its 360 degree upper limit");
    Require(plan->jointPositionRadians.back() >= models::hanwha::kHcr12a.joints[5].minPositionRadians &&
        plan->jointPositionRadians.back() <= models::hanwha::kHcr12a.joints[5].maxPositionRadians,
        "wrist target stays inside the J6 hard limits");
}
}

int main()
{
    try
    {
        CheckWristTargetChangesOnlyJ6();
        CheckWristTargetRespectsJ6Limits();
        std::cout << "Robot panel motion planning checks passed\n";
        return 0;
    }
    catch (const std::exception& exception)
    {
        std::cerr << exception.what() << '\n';
        return 1;
    }
}
