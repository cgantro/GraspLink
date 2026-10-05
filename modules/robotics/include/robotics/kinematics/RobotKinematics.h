#pragma once

#include "robotics/core/ControlTypes.h"
#include "robotics/models/RobotSpecification.h"

#include <vector>

namespace grasplink::robotics::kinematics
{

struct RobotKinematicState
{
    // 좌표: 각 관절의 Local 회전. GLB의 초기 관절 회전은 0이어야 함.
    std::vector<models::QuaternionWxyz> jointLocalRotations;
    // 좌표: Robot base 기준 관절 중심 [m]과 누적 회전.
    std::vector<models::Pose3> linkPosesInBaseFrame;
    // 마지막 joint pose에 RobotSpecification의 ToolFrame offset을 적용한 base-frame pose.
    // Controller의 tcpPose feedback과 별도 FK 출력이다.
    models::Pose3 toolFrameInBaseFrame{};
    // RobotSpecification에 ToolFrame 정의가 있으면 true.
    bool toolFrameValid = false;
};

class RobotKinematics final
{
public:
    // 참조: specification과 joints 배열은 이 객체보다 오래 살아야 한다.
    // 지원: joints 순서 직렬 체인, base-frame bind pivot, identity joint bind rotation.
    explicit RobotKinematics(const models::RobotSpecification& specification);

    // 입력: Controller 순서 J1..Jn 관절각 [rad]. 출력: Robot base 기준 pose.
    // 수명: 반환값은 이 객체가 소유하며, 다음 Update에서 덮어씀.
    const RobotKinematicState& Update(const RobotState& state);

private:
    const models::RobotSpecification& specification_;
    RobotKinematicState state_;
};

}
