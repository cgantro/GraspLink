#pragma once

#include "Entity.h"
#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/models/RobotSpecification.h"

#include <vector>

namespace grasplink::viewer::robotics
{

/**
 * @brief FK 관절 Local 회전을 검증된 GLB Entity 계층에 반영한다.
 * @details 모델의 joint 이름으로 authored GLB 노드를 연결하고 base-frame bind pivot, 직렬 부모 관계,
 * 항등 bind 회전을 확인한다. 배치용 robot root는 검사 대상에서 빼며, Scene의 계층 변환이 FK base pose를
 * Scene 좌표로 옮긴다. Entity는 Scene 소유이므로 Scene 제거 뒤 적용은 실패한다.
 */
class RobotTransformAdapter
{
public:
    /**
     * @brief Specification의 관절을 authored GLB Entity에 연결하고 bind 상태를 검사한다.
     * @param robotRoot Scene에서 배치될 모델 base Entity.
     * @param specification GLB 관절 이름, 순서와 base-frame bind pivot을 제공하는 모델 참조.
     * @throws std::runtime_error root가 없거나 GLB에 관절이 없을 때.
     * @throws std::invalid_argument 관절 계층 또는 bind 변환이 FK 모델과 맞지 않을 때.
     */
    RobotTransformAdapter(
        const Entity& robotRoot,
        const ::grasplink::robotics::models::RobotSpecification& specification);

    /**
     * @brief FK local 회전을 대응 GLB 관절에 적용한다.
     * @param state 같은 RobotSpecification 순서로 계산한 FK 결과.
     * @throws std::invalid_argument 관절 회전 수가 연결된 Entity 수와 다를 때.
     * @throws std::runtime_error Scene이 제거되어 Entity가 유효하지 않을 때.
     */
    void Apply(const ::grasplink::robotics::kinematics::RobotKinematicState& state);

private:
    struct JointBinding
    {
        Entity entity;
    };

    /// 모델 joint 순서에 대응하는 Scene 소유 Entity 목록.
    std::vector<JointBinding> joints_;
};

} // namespace grasplink::viewer::robotics
