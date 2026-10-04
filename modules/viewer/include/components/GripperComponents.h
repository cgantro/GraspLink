#pragma once

/**
 * @file GripperComponents.h
 * @brief Viewer/ECS에서 gripper 관련 Entity를 표시하기 위한 보조 Component.
 *
 * @details
 * 용어:
 * - Gripper: 물체를 잡기 위해 열리고 닫히는 말단장치.
 * - Finger: 물체에 직접 닿는 좌/우 손가락 부품.
 * - Mirrored motion: 좌우 손가락이 거울처럼 반대 방향으로 움직이는 관계.
 * - Grabbable: 나중에 collision/grasp 대상이 될 수 있다고 표시된 물체.
 *
 * 아래 값들은 현재 Viewer 쪽 legacy 표현이며, 실제 2F-85 Controller 모델의 source of truth가 아니다.
 */

/**
 * @brief Viewer에서 gripper가 얼마나 열려 있는지 간단히 표현하던 legacy Component.
 *
 * @details
 * `openness`는 현재 물리적으로 정의된 공식 값이 아니다.
 * 즉 다음 중 어느 것과도 1:1로 동일하다고 가정하면 안 된다.
 * - Robotiq rPR 0..255
 * - fingertip opening [m/mm]
 * - master linkage angle [rad]
 *
 * 향후 SimGripperController가 들어오면 Controller state에서 Viewer 표시값을 파생하거나
 * 이 Component를 제거하는 방향이 적절하다.
 */
struct Gripper
{
    /** @brief legacy normalized openness 값. 공식 물리 단위/범위가 고정되지 않았으므로 제어 입력으로 사용하지 않는다. */
    float openness = 0.0F;
};

/**
 * @brief 좌/우 finger가 서로 반대 방향으로 움직이는 부호 관계를 표현하던 legacy 보조 Component.
 *
 * @details
 * 예를 들어 direction=+1/-1을 사용하면 같은 입력값으로 좌우가 반대 방향으로 움직이게 만들 수 있다.
 * 현재 controller-ready 2F-85의 실제 free-space linkage 관계는
 * `robotics::models::robotiq::kTwoF85Joints`의 `masterMultiplier`가 기준이다.
 */
struct GripperFinger
{
    /** @brief mirrored motion에 사용할 무차원 부호/배율. 일반적으로 +1 또는 -1. */
    float direction = 1.0F;
};

/**
 * @brief 이 Entity가 향후 물리 충돌/파지 대상이 될 수 있음을 표시하는 값 없는 ECS tag.
 *
 * @details
 * Tag는 데이터를 저장하지 않고 "이 Entity는 이런 성격을 가진다"는 표식 역할만 한다.
 * 실제 collision detection, grasp 판정, attach/detach는 Physics 계층에서 구현해야 한다.
 */
struct Grabbable
{
};
