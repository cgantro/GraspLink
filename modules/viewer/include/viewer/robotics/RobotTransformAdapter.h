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
/*
 * [추가 용어 설명]
 * - Adapter: 서로 다른 두 표현 사이를 변환해 연결하는 얇은 계층.
 * - Entity: Flecs ECS 안에서 하나의 객체/Node를 가리키는 handle.
 * - Transform: 위치(Position), 회전(Rotation), 크기(Scale)를 묶은 공간 상태.
 * - Local Transform: 부모 Node를 기준으로 한 transform.
 * - World Transform: Scene 전체 원점을 기준으로 계산된 최종 transform.
 * - Bind rotation: GLB를 처음 불러왔을 때 Joint Node가 가진 기준 회전.
 * - Identity rotation: 회전이 없는 상태. Euler로는 (0,0,0), quaternion으로는 (0,0,0,1).
 *
 * 데이터 흐름:
 * RobotState::jointPositionRadians[i] [rad]
 *   + JointSpecification::axis
 *   -> angle-axis 회전
 *   -> Flecs Joint Entity Local Rotation
 *   -> TransformSystem이 World Matrix 계산
 */
class RobotTransformAdapter
{
public:
    /**
     * @brief RobotSpecification의 Joint 이름을 이용해 GLB/Flecs Joint Entity를 찾아 연결한다.
     * @param robotRoot 생성된 robot hierarchy의 root Entity.
     * @param specification Joint 이름/축/개수를 제공하는 모델 정의.
     */
    RobotTransformAdapter(
        const Entity& robotRoot,
        const ::grasplink::robotics::models::RobotSpecification& specification);

    /**
     * @brief RobotState의 J1..Jn 절대 관절각 [rad]을 화면상의 Joint Local Rotation에 반영한다.
     * @note 이 함수는 target을 계산하거나 limit을 검사하는 Controller가 아니다. 이미 계산된 state를 표시만 한다.
     */
    void Apply(const ::grasplink::robotics::RobotState& state);

private:
    struct JointBinding
    {
        // 실제 화면에서 회전을 수정할 Flecs Joint Entity handle.
        Entity entity;

        // 해당 Joint local frame의 회전축. 예: {0,1,0}=local +Y.
        ::grasplink::robotics::models::Axis3 axis;
    };

    // RobotSpecification의 J1..Jn 순서를 그대로 유지하는 Entity/axis 연결 목록.
    std::vector<JointBinding> joints_;
};

} // namespace grasplink::viewer::robotics
