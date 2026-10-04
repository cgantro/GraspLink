#pragma once

/**
 * @brief 그리퍼의 논리적 개방 상태를 표현하는 ECS Component.
 *
 * @note 현재 값은 단순 상태 표현이며 실제 2F-85의 0~85 mm opening과 직접 연결되지 않았다.
 * @todo [FUTURE] Robotiq 2F-85 제어 모델이 들어오면 단위를 meter 또는 mm로 명확히 고정하고
 *       속도/접촉 상태를 별도 Component로 분리한다.
 */
/*
 * [추가 현재 구조/용어 설명]
 * - Gripper: 로봇 끝단에서 물체를 잡는 말단장치.
 * - Openness: 집게가 얼마나 열렸는지를 나타내려는 논리 상태값.
 * - ECS Component: Entity에 붙는 작은 데이터 조각.
 *
 * [현재 구조 주의]
 * 이 기존 Component는 controller 계층 도입 이전의 legacy viewer 상태다.
 * 현재 Robotiq 2F-85의 command/source-of-truth는 GripperCommand, GripperSpecification,
 * models/robotiq/TwoF85.h 쪽이며 `openness`를 rPR, mm opening, master q로 간주하면 안 된다.
 */
struct Gripper
{
    // 현재 legacy normalized openness 값. 물리 단위/공식 범위가 고정되지 않아 제어 입력으로 사용하지 않는다.
    float openness = 0.0F;
};

/**
 * @brief 좌/우 손가락의 이동 방향을 구분하기 위한 보조 Component.
 * @todo [FUTURE] 현재 GLB FingerJoint는 실제 linkage pivot과 다르므로 정밀 기구학을 구현할 때 재설계한다.
 */
/*
 * [추가 현재 구조/용어 설명]
 * - Mirrored motion: 좌/우 대칭 부품이 서로 반대 부호로 움직이는 관계.
 * - Linkage: 여러 링크와 관절이 연결되어 한 구동 입력에 함께 움직이는 기구.
 *
 * [현재 구조 주의]
 * 위 기존 TODO는 controller-ready 2F-85 GLB 이전에 작성된 기록이다.
 * 현재 asset은 Outer/Inner/FingerTip Joint를 분리했으며 실제 free-space 연동 관계는
 * models::robotiq::kTwoF85Joints의 masterMultiplier가 기준이다.
 */
struct GripperFinger
{
    // legacy mirrored motion 부호/배율. 현재 2F-85 linkage source-of-truth는 이 값이 아니다.
    float direction = 1.0F;
};

/** @brief 이 태그가 붙은 Entity가 grasp 대상이 될 수 있음을 표시한다. */
/* Grabbable은 실제 충돌/파지/attach 로직을 수행하지 않고 대상 식별만 하는 값 없는 tag다. */
struct Grabbable
{
};
