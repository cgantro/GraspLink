#include "simulation/SimulationSceneBuilder.h"

#include "simulation/components/PhysicsComponents.h"

namespace grasplink::simulation
{
void SimulationSceneBuilder::ConfigureFloor(Entity& floor)
{
    // plane.glb 범위: X/Z ±3 m, 표면 Y=0. Collider 반크기와 중심은 Entity 기준 [m].
    // Static 바닥은 Jolt가 움직이지 않는다. Collider 상면은 Local Y=0.005 m, 중심은 아래에 둔다.
    floor.set<RigidBody>(RigidBody{grasplink::physics::BodyMotionType::Static,
        grasplink::physics::CollisionLayer::Environment})
        .set<Colliders>(Colliders{{physics_colliders::Box(
            {3.0F, 0.02F, 3.0F}, {0.0F, -0.015F, 0.0F})}});
}
}
