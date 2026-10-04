#pragma once

#include "Entity.h"
#include "robotics/core/ControlTypes.h"
#include "robotics/models/RobotSpecification.h"

#include <vector>

namespace grasplink::viewer::robotics
{

/**
 * @brief Controller가 계산한 관절각을 화면 속 GLB/Flecs 관절 회전으로 바꾸는 Viewer 전용 adapter.
 *
 * @details
 * 이 클래스는 로봇을 "어떻게 움직일지" 결정하지 않는다.
 * Controller가 이미 계산한 RobotState를 받아서 "그 상태를 화면에 어떻게 보여줄지"만 담당한다.
 *
 * 로보틱스/그래픽스 용어:
 * - Adapter: 서로 다른 두 계층의 데이터 형식을 이어주는 변환 객체.
 * - Binding: 논리적인 Robot Joint와 실제 화면상의 Entity를 1:1로 연결해 둔 관계.
 * - Bind pose: GLB를 처음 로드했을 때의 기준 자세.
 * - Pivot: 관절이 회전하는 중심점. 현재 GLB Node translation/hierarchy에 이미 들어 있다.
 * - Local rotation: 부모 Link를 기준으로 한 현재 Joint의 회전.
 * - World transform: 모든 부모 transform까지 누적한 최종 화면 위치/회전.
 * - Quaternion: 회전을 4개 성분으로 표현하는 방식. 회전 합성에 유리하다.
 * - Euler angle: X/Y/Z축 회전각 3개로 회전을 표현하는 방식. 현재 ECS 저장 형식이므로 중간에 변환한다.
 * - Identity rotation: 회전이 전혀 없는 상태.
 *
 * 변환 흐름:
 * `RobotState q[rad] + Joint axis`
 * -> `angleAxis(q, axis)` quaternion
 * -> Euler radians
 * -> Entity::SetLocalRotation()
 * -> TransformSystem의 Local/World matrix
 * -> 최종 GLB Link 렌더링.
 *
 * Joint pivot은 이미 GLB hierarchy에 있으므로 이 Adapter가 bindPivotMeters를 매 frame 다시 더하지 않는다.
 */
class RobotTransformAdapter
{
public:
    /**
     * @brief RobotSpecification의 각 Joint 이름을 실제 GLB/Flecs Entity와 연결(binding)한다.
     * @param robotRoot 로드된 robot hierarchy의 최상위 Entity.
     * @param specification Joint 이름/축/개수를 제공하는 모델 사양.
     * @throws std::runtime_error root가 invalid이거나 Joint Entity를 못 찾거나 bind rotation 계약이 깨진 경우.
     * @throws std::invalid_argument specification이 비어 있는 경우.
     *
     * @details
     * 예를 들어 specification의 "J2"를 보고 robotRoot 아래에서 이름이 "J2"인 Entity를 찾아 저장한다.
     * 이후 Apply()에서는 매번 이름 검색을 하지 않고 저장된 binding을 바로 사용한다.
     */
    RobotTransformAdapter(
        const Entity& robotRoot,
        const ::grasplink::robotics::models::RobotSpecification& specification);

    /**
     * @brief RobotState의 J1..Jn 관절각을 각 GLB Joint Entity의 Local Rotation에 적용한다.
     * @param state Controller가 계산한 현재 RobotState. jointPositionRadians 단위는 [rad].
     *
     * @details
     * `state.valid=false`이면 화면에 반영하지 않는다.
     * 관절각 자체는 Controller의 source of truth이며, Adapter는 값을 제한하거나 다시 계산하지 않는다.
     *
     * @throws std::invalid_argument state의 joint 수가 specification과 다르거나 angle이 NaN/Inf인 경우.
     */
    void Apply(const ::grasplink::robotics::RobotState& state);

private:
    /**
     * @brief 모델의 Joint 하나와 화면상의 Entity 하나를 연결해 둔 내부 binding.
     *
     * @details
     * `entity`는 실제 화면 transform을 수정할 대상이고,
     * `axis`는 그 Entity가 어떤 local 축을 중심으로 회전해야 하는지 나타낸다.
     */
    struct JointBinding
    {
        /** @brief GLB/Flecs hierarchy에서 Local Rotation을 수정할 Entity handle. Entity 자체의 소유권은 없다. */
        Entity entity;

        /** @brief 해당 Joint local frame의 회전축. 물리 단위 없는 방향벡터. */
        ::grasplink::robotics::models::Axis3 axis;
    };

    /** @brief RobotSpecification의 J1..Jn 순서를 그대로 유지한 JointBinding 목록. */
    std::vector<JointBinding> joints_;
};

} // namespace grasplink::viewer::robotics
