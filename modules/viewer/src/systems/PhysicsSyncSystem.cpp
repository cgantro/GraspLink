#include "systems/PhysicsSyncSystem.h"

#include "components/PhysicsComponents.h"
#include "components/TransformComponents.h"
#include "PhysicsWorld.h"

#include <glm/gtx/quaternion.hpp>


PhysicsSyncSystem::PhysicsSyncSystem(
    flecs::world& world,grasplink::physics::PhysicsWorld& physicsWorld)
    : m_World(world),m_PhysicsWorld(physicsWorld)
{
}


void PhysicsSyncSystem::SyncPhysicsToEntity()
{
    m_World.each<PhysicsBodyComponent>(
        [this](flecs::entity entity, const PhysicsBodyComponent& physicsBody)
        {
            if (physicsBody.syncMode != PhysicsSyncMode::PhysicsToEntity)
                return;

            if (!m_PhysicsWorld.IsBodyValid(physicsBody.body))
                return;

            const auto transform = m_PhysicsWorld.GetBodyTransform(physicsBody.body);

            entity.set<Position, Local>(Position{transform.position});
            entity.set<Rotation, Local>(Rotation{glm::eulerAngles(transform.rotation)});
        });
}