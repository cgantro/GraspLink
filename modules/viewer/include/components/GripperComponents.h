#pragma once

/**
 * @brief Viewer/ECS에서 사용하는 간단한 논리적 gripper 개방 상태 Component.
 *
 * @details
 * 현재 `openness`는 legacy/viewer-side 상태 표현이며 Robotiq 2F-85의 실제 rPR(0..255),
 * fingertip opening [m], master linkage angle [rad] 중 어느 하나로 공식 정의된 값이 아니다.
 * 따라서 robotics controller의 source of truth로 사용하면 안 된다.
 *
 * 향후 SimGripperController가 도입되면 Controller의 GripperState/2F-85 model specification을 기준으로
 * Viewer가 필요한 표시 상태만 파생하거나 이 Component 자체를 제거하는 것이 적절하다.
 */
struct Gripper
{
    /** @brief 현재 legacy normalized openness 값. 물리 단위/범위가 아직 고정되지 않았으므로 제어 입력으로 사용하지 않는다. */
    float openness = 0.0F;
};

/**
 * @brief 좌/우 또는 mirrored gripper part의 방향 부호를 표시하는 legacy 보조 Component.
 *
 * @details
 * `direction=+1/-1`처럼 대칭 움직임의 부호를 표현하기 위한 값이다.
 * controller-ready 2F-85 모델의 실제 linkage 관계는 robotics::models::robotiq::kTwoF85Joints의
 * masterMultiplier가 source of truth다.
 */
struct GripperFinger
{
    /** @brief mirrored motion에 사용할 무차원 부호/배율. 일반적으로 +1 또는 -1. */
    float direction = 1.0F;
};

/**
 * @brief 이 Entity가 향후 grasp/collision 대상이 될 수 있음을 표시하는 값 없는 ECS tag.
 *
 * @note 실제 파지 판정이나 attach/detach physics 로직을 수행하는 Component는 아니다.
 */
struct Grabbable
{
};
