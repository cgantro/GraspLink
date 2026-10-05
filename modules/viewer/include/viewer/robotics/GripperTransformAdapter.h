#pragma once

#include "Entity.h"
#include "robotics/kinematics/GripperKinematics.h"
#include "robotics/models/GripperSpecification.h"

#include <vector>

namespace grasplink::viewer::robotics
{

/**
 * @brief Gripper 분기 관절의 Local 회전 변화를 authored GLB 계층에 적용한다.
 * @details specification 순서의 이름으로 root 하위 joint를 찾고 생성 시 bind 회전과 root까지의
 * 부모 경로를 저장한다. Apply는 저장한 bind quaternion에 delta quaternion을 오른쪽으로 곱해 Local
 * quaternion 회전만 설정한다. 원본 bind 위치·크기와 Gripper 장착 변환은 그대로 둔다. World 위치를 계산하지
 * 않으며 부모/팔/root의 이동·회전은 기존 TransformSystem이 계층으로 전달한다. Entity와 Scene은 빌려 쓰므로
 * Scene 제거 또는 joint 중간 계층 변경 뒤 Apply는 실패한다.
 */
class GripperTransformAdapter final
{
public:
    /**
     * @brief root 아래의 사양 관절을 찾아 bind pose와 부모 경로를 연결한다.
     * @param gripperRoot GLB Gripper node에 해당하는 Entity. 비항등 Local 장착 변환을 보존한다.
     * @param specification joint 이름과 specification 순서를 제공하는 모델. 생성 중에만 읽는다.
     * @throws std::runtime_error root나 필수 joint가 없거나 중복 이름·잘못된 부모 경로·삭제된 Entity가 있을 때.
     * @throws std::invalid_argument 관절 bind TRS가 비유한 값이거나 계층 scale이 단위가 아닐 때.
     */
    GripperTransformAdapter(
        const Entity& gripperRoot,
        const ::grasplink::robotics::models::GripperSpecification& specification);

    /**
     * @brief 결과를 모든 관절에 검증 후 적용한다.
     * @param state specification 순서의 유한한 관절각과 axis-angle delta quaternion.
     * @throws std::invalid_argument 결과 개수, 각도 또는 회전이 유효하지 않을 때.
     * @throws std::runtime_error Scene에서 Entity가 삭제됐거나 저장한 부모 경로가 달라졌을 때.
     * @details 유효성 검사와 변환 계산을 먼저 모두 마친 뒤에만 Local 회전을 쓴다. bind quaternion에서
     * 매번 계산하므로 반복 Apply가 누적 회전 오차를 만들지 않는다.
     */
    void Apply(const ::grasplink::robotics::kinematics::GripperKinematicState& state);

private:
    struct JointBinding
    {
        Entity entity;
        ::grasplink::robotics::models::QuaternionWxyz bindRotation;
        // 첫 원소는 관절 자체이며 마지막 원소는 gripperRoot다. 중간 GLB node도 경로 검증에 포함한다.
        std::vector<Entity> ancestry;
    };

    Entity gripperRoot_;
    std::vector<JointBinding> joints_;
    // 전체 관절 검증 후 반영하는 준비 버퍼. Fixed Update에서 반복 할당하지 않는다.
    std::vector<glm::quat> preparedRotations_;
};

} // namespace grasplink::viewer::robotics
