#pragma once

#include <flecs.h>

namespace grasplink::physics
{
class PhysicsWorld;
}

/*
 * Jolt Physics Body의 Transform을 Flecs Entity에 반영한다.
 *
 * Physics 계산 자체는 하지 않는다.
 * PhysicsWorld와 ECS 사이를 연결하는 역할만 한다.
 */
class PhysicsSyncSystem final
{
public:
    PhysicsSyncSystem(flecs::world& world, grasplink::physics::PhysicsWorld& physicsWorld);

    void SyncPhysicsToEntity();

private:
    flecs::world& m_World;
    grasplink::physics::PhysicsWorld& m_PhysicsWorld;
};