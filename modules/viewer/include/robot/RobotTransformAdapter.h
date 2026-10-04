#pragma once

#include "Entity.h"
#include "control/ControlTypes.h"
#include "control/specs/DeviceSpecifications.h"

#include <vector>

/**
 * @brief Controller의 joint state를 Viewer/Flecs Joint Entity transform으로 반영한다.
 *
 * @details
 * 이 클래스는 rendering adapter다. target 추종, limit, IK/FK, 통신 상태를 계산하지 않는다.
 * RobotSpecification의 joint name/axis를 사용해 GLB Joint Node를 찾고 현재 상태를 시각화한다.
 * controller-ready GLB의 Joint bind rotation은 identity라는 asset contract를 전제로 한다.
 */
class RobotTransformAdapter
{
public:
    RobotTransformAdapter(
        const Entity& robotRoot,
        const control::specs::RobotSpecification& specification);

    /** @brief 유효한 RobotState의 관절 각도를 J1...Jn Entity에 적용한다. */
    void Apply(const control::RobotState& state);

private:
    struct JointBinding
    {
        Entity entity;
        control::specs::Axis3 axis;
    };

    std::vector<JointBinding> joints_;
};
