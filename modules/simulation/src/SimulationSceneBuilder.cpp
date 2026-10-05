#include "simulation/SimulationSceneBuilder.h"

#include "simulation/components/PhysicsComponents.h"

namespace grasplink::simulation
{
void SimulationSceneBuilder::ConfigureFloor(Entity& floor)
{
    // plane.glb 렌더 범위는 X/Z ±3 m, 표면 Y=0이며 두께는 1 mm다. 충돌 치수는 GLB에서 추출하지 않는다.
    // 별도 Box는 Entity 원점 기준 반크기 (3, 0.02, 3) m, 중심 Y=-0.015 m라 상면이 Y=0.005 m다.
    // 시각 Mesh보다 두꺼운 접촉 부피를 아래로 배치해 표면을 받친다. Static이라 Jolt는 이동시키지 않는다.
    floor.set<RigidBody>(RigidBody{grasplink::physics::BodyMotionType::Static,
        grasplink::physics::CollisionLayer::Environment})
        .set<Colliders>(Colliders{{physics_colliders::Box(
            {3.0F, 0.02F, 3.0F}, {0.0F, -0.015F, 0.0F})}});
}
}
