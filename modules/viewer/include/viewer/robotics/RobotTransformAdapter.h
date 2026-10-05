#pragma once

#include "Entity.h"
#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/models/RobotSpecification.h"

#include <vector>

namespace grasplink::viewer::robotics
{

// 역할: FK 관절 회전을 검증된 GLB Entity 계층에 반영.
// 소유: 관절 Entity는 Scene이 소유. Scene 제거 후 Apply는 오류를 반환.
class RobotTransformAdapter
{
public:
    // GLB bind pivot, identity local rotation, 직렬 joint hierarchy를 확인하고 이름을 연결한다.
    // Robot root의 transform은 모델 base pose이므로 bind 검사에서 제외한다.
    RobotTransformAdapter(
        const Entity& robotRoot,
        const ::grasplink::robotics::models::RobotSpecification& specification);

    // 입력: RobotKinematicState의 joint local 회전을 GLB Entity에 적용한다.
    void Apply(const ::grasplink::robotics::kinematics::RobotKinematicState& state);

private:
    struct JointBinding
    {
        Entity entity;
    };

    // 모델 Joint 순서와 같은 Entity 목록
    std::vector<JointBinding> joints_;
};

} // namespace grasplink::viewer::robotics
