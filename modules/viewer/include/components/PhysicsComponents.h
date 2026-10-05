#pragma once

#include "PhysicsTypes.h"

#include <glm/gtc/quaternion.hpp>

/**
 * @brief 물체의 움직임 방식 설정
 *
 * - Dynamic: Jolt가 중력·충돌 계산
 * - Kinematic: Controller가 Entity를 움직이고 PhysicsSystemModule이 Jolt에 자세 전달
 * - Static: 바닥처럼 고정
 * Body 설정만 저장. Jolt Body는 PhysicsSystemModule이 관리
 */
struct RigidBody
{
    grasplink::physics::BodyMotionType motionType =
        grasplink::physics::BodyMotionType::Dynamic;
};

/**
 * @brief Box 충돌 범위와 Entity 기준 중심 위치 설정
 *
 * halfExtentsMeters: 각 축의 반 길이 (미터). `{0.5, 0.5, 0.5}` → 한 변 1m
 * Entity Scale: Collider 크기에 자동 반영되지 않음
 * localPositionMeters / localRotation: Entity 원점에서 Collider 중심까지의 Local 이동·회전
 * 생성 조건: RigidBody와 BoxCollider 모두 필요
 */
struct BoxCollider
{
    glm::vec3 halfExtentsMeters{0.5F};
    glm::vec3 localPositionMeters{0.0F};
    glm::quat localRotation{1.0F, 0.0F, 0.0F, 0.0F};
};
