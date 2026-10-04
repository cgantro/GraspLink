#pragma once

#include "Entity.h"
#include "robotics/core/ControlTypes.h"
#include "robotics/models/RobotSpecification.h"

#include <vector>

namespace grasplink::viewer::robotics
{

/**
 * @brief Controller joint state를 Viewer/Flecs Joint Entity transform으로 반영하는 adapter.
 *
 * target 추종, limit, IK/FK, 통신 상태를 계산하지 않는다.
 * controller-ready GLB의 Joint bind rotation은 identity라는 asset contract를 전제로 한다.
 */
class RobotTransformAdapter
{
public:
    RobotTransformAdapter(
        const Entity& robotRoot,
        const ::grasplink::robotics::models::RobotSpecification& specification);

    void Apply(const ::grasplink::robotics::RobotState& state);

private:
    struct JointBinding
    {
        Entity entity;
        ::grasplink::robotics::models::Axis3 axis;
    };

    std::vector<JointBinding> joints_;
};

} // namespace grasplink::viewer::robotics
