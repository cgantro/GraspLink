#pragma once

#include "PhysicsTypes.h"

#include <cstdint>

/*
 * Physics와 Entity 중 어느 쪽 Transform을 기준으로 할지 정한다.
 */
enum class PhysicsSyncMode : std::uint8_t
{
    PhysicsToEntity,
    EntityToPhysics,
    None
};

/*
 * Flecs Entity와 Physics Body를 연결한다.
 *
 * Entity
 *   -> PhysicsBodyComponent
 *   -> PhysicsBodyHandle
 *   -> PhysicsWorld
 *   -> Jolt Body
 */
struct PhysicsBodyComponent
{
    grasplink::physics::PhysicsBodyHandle body;
    PhysicsSyncMode syncMode = PhysicsSyncMode::PhysicsToEntity;
};