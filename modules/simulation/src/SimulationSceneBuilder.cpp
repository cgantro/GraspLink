#include "simulation/SimulationSceneBuilder.h"

#include "scene/Entity.h"
#include "simulation/components/PhysicsComponents.h"

namespace grasplink::simulation
{
void ConfigureStaticFloor(
    grasplink::scene::Entity& floor,
    const glm::vec3& halfExtentsMeters,
    const glm::vec3& localCenterOffsetMeters)
{
    floor.set<RigidBody>({grasplink::physics::BodyMotionType::Static,
        grasplink::physics::CollisionLayer::Environment})
        .set<Colliders>({{physics_colliders::Box(halfExtentsMeters, localCenterOffsetMeters)}});
}
} // namespace grasplink::simulation
