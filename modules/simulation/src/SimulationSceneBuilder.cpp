#include "simulation/SimulationSceneBuilder.h"

#include "simulation/components/PhysicsComponents.h"

namespace grasplink::simulation
{
void SimulationSceneBuilder::ConfigureFloor(Entity& floor)
{
    // plane.glb는 X와 Z 방향으로 원점에서 각각 3 m까지 그려지고 표면은 Y=0에 있다. Mesh 두께는 1 mm지만 충돌 Box 크기는 이 GLB에서 읽지 않고 별도로 정한다.
    // 충돌 Box는 Entity 원점 기준 반크기 (3, 0.02, 3) m이고 중심을 Y=-0.015 m에 둔다. 따라서 Box 윗면은 Y=0.005 m로 렌더 바닥보다 5 mm 위에 놓인다.
    // 1 mm 시각 Mesh 대신 40 mm 두께의 충돌 부피를 아래로 늘려 물체가 바닥과 안정적으로 접촉하게 한다. Static Body이므로 Jolt가 위치를 움직이지 않는다.
    floor.set<RigidBody>(RigidBody{grasplink::physics::BodyMotionType::Static,
        grasplink::physics::CollisionLayer::Environment})
        .set<Colliders>(Colliders{{physics_colliders::Box(
            {3.0F, 0.02F, 3.0F}, {0.0F, -0.015F, 0.0F})}});
}
}
