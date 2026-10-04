#pragma once

#include "Entity.h"
#include "robotics/core/ControlTypes.h"
#include "robotics/models/RobotSpecification.h"

#include <vector>

namespace grasplink::viewer::robotics
{

/**
 * @brief Robotics Controller의 joint state를 Viewer/Flecs Joint Entity transform으로 변환하는 시각화 adapter.
 *
 * @details
 * 이 클래스는 제어기가 아니다. target 추종, angle/velocity limit, FK/IK, trajectory, 통신 상태를 계산하지 않는다.
 * 입력은 이미 계산된 RobotState이고, 출력은 GLB hierarchy를 표현하는 Flecs Entity의 Local Rotation이다.
 *
 * HCR-12A controller-ready GLB에 대한 변환 흐름:
 *
 * `RobotState::jointPositionRadians[i]` [rad]
 *   + `JointSpecification::axis` [joint-local unit vector]
 *   -> `glm::angleAxis(q, axis)` quaternion
 *   -> 현재 ECS Rotation 저장 형식인 Euler radians로 변환
 *   -> Entity::SetLocalRotation()
 *   -> TransformSystemModule에서 `Local = Translation * Rotation * Scale`
 *   -> `World = ParentWorld * Local`
 *
 * 관절 회전 중심은 GLB Node hierarchy/translation에 이미 들어 있다. 따라서 이 Adapter는
 * JointSpecification::bindPivotMeters를 매 frame translation으로 다시 적용하지 않는다.
 *
 * @note controller-ready GLB contract상 moving Joint Node(J1..J6)의 bind rotation은 identity여야 한다.
 *       생성자에서 이를 검사해 옛 asset의 rotation compensation이 섞이는 것을 조기에 막는다.
 * @warning 현재 ECS Rotation이 Euler vec3이므로 quaternion을 다시 Euler로 변환한다.
 *          향후 Transform rotation 저장을 quaternion으로 바꾸면 이 중간 변환을 제거하는 것이 바람직하다.
 */
class RobotTransformAdapter
{
public:
    /**
     * @brief GLB hierarchy의 Joint Entity와 RobotSpecification의 joint axis를 이름 기준으로 binding한다.
     * @param robotRoot PrefabFactory가 만든 robot hierarchy root Entity.
     * @param specification Joint 이름/축/개수를 제공하는 robot model specification.
     * @throws std::runtime_error root가 invalid이거나 joint를 찾지 못했거나 bind rotation이 identity가 아닌 경우.
     * @throws std::invalid_argument specification이 비어 있는 경우.
     */
    RobotTransformAdapter(
        const Entity& robotRoot,
        const ::grasplink::robotics::models::RobotSpecification& specification);

    /**
     * @brief RobotState의 J1..Jn 절대 각도를 각 bound Joint Entity의 Local Rotation에 적용한다.
     * @param state jointPositionRadians가 specification 순서와 같은 RobotState. 각도 단위 [rad].
     *
     * @note state.valid=false이면 아무 것도 적용하지 않는다.
     * @throws std::invalid_argument state joint 개수가 binding 개수와 다르거나 angle이 finite하지 않은 경우.
     */
    void Apply(const ::grasplink::robotics::RobotState& state);

private:
    /** @brief 하나의 model joint와 화면상의 Flecs Joint Entity를 연결하는 내부 binding. */
    struct JointBinding
    {
        /** @brief GLB/Flecs hierarchy에서 실제 Local Rotation을 수정할 non-owning Entity handle. */
        Entity entity;

        /** @brief 해당 Joint local frame의 회전축. Apply 시 glm::vec3로 변환하고 normalize한다. */
        ::grasplink::robotics::models::Axis3 axis;
    };

    /** @brief RobotSpecification의 J1..Jn 순서를 그대로 유지하는 joint binding 배열. */
    std::vector<JointBinding> joints_;
};

} // namespace grasplink::viewer::robotics
