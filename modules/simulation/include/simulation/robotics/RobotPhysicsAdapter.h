#pragma once

#include "Entity.h"
#include "assets/GraphicsTypes.h"
#include "robotics/models/RobotSpecification.h"

#include <vector>

class Scene;

namespace grasplink::robotics::kinematics
{
struct RobotKinematicState;
}

namespace grasplink::simulation
{

// GLB link 형상으로 Kinematic 충돌 Entity를 만들고 FK 자세를 전달한다.
class RobotPhysicsAdapter final
{
public:
    // Scene이 proxy Entity를 소유한다. Apply 중에도 Scene과 robotRoot가 살아 있어야 한다.
    RobotPhysicsAdapter(
        Scene& scene,
        const Entity& robotRoot,
        const grasplink::robotics::models::RobotSpecification& specification,
        const ModelResource& model);

    // 입력: 로봇 Base 기준 FK 자세 [m, quaternion]. robotRoot 자식의 Local 자세로 저장한다.
    void Apply(const grasplink::robotics::kinematics::RobotKinematicState& state);

private:
    struct LinkBinding
    {
        Entity entity;
        std::size_t jointIndex = 0;
    };

    std::vector<LinkBinding> links_;
};

}
