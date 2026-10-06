#include "robotics/backends/simulation/SimRobotController.h"
#include "robotics/models/hanwha/Hcr12a.h"
#include "robotics/runtime/FixedControlLoop.h"
#include "TestSupport.h"

#include <iostream>
#include <limits>

using namespace grasplink::robotics;

/**
 * @brief FixedControlLoop와 시뮬레이션 Controller의 시간·명령 계약을 확인한다.
 * @details 고정 간격 누적/초과 시간 제한, 잘못된 경과 시간 무시, 관절 속도 제한과 범위 거부를 검사한다.
 * 또한 정지·재목표 동작과 미구현 TCP feedback/linear trajectory 경계를 고정한다.
 */
int main()
{
    try
    {
        // 입력 시간은 초 단위다. 250 Hz 설정은 4 ms 고정 step으로 나뉘며, 긴 정지 뒤에도 한 번에 최대 100 ms만 누적한다.
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
        // 목표 회전은 rad, 최대 회전 속도는 rad/s로 지정한다. 짧은 한 번의 업데이트에서 속도 한도를 넘지 않는 만큼만 각도가 변하는지 확인한다.
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
