#pragma once

/**
 * @brief 그리퍼의 논리적 개방 상태를 표현하는 ECS Component.
 *
 * @note 현재 값은 단순 상태 표현이며 실제 2F-85의 0~85 mm opening과 직접 연결되지 않았다.
 * @todo [FUTURE] Robotiq 2F-85 제어 모델이 들어오면 단위를 meter 또는 mm로 명확히 고정하고
 *       속도/접촉 상태를 별도 Component로 분리한다.
 */
struct Gripper
{
    float openness = 0.0F;
};

/**
 * @brief 좌/우 손가락의 이동 방향을 구분하기 위한 보조 Component.
 * @todo [FUTURE] 현재 GLB FingerJoint는 실제 linkage pivot과 다르므로 정밀 기구학을 구현할 때 재설계한다.
 */
struct GripperFinger
{
    float direction = 1.0F;
};

/** @brief 이 태그가 붙은 Entity가 grasp 대상이 될 수 있음을 표시한다. */
struct Grabbable
{
};
