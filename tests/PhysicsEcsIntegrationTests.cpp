#include "Entity.h"
#include "PhysicsWorld.h"
#include "components/TransformComponents.h"
#include "scene/SceneManager.h"
#include "simulation/components/PhysicsComponents.h"
#include "simulation/systems/PhysicsSystemModule.h"
#include "systems/TransformSystemModule.h"
#include "TestSupport.h"

#include <iostream>

using grasplink::physics::BodyMotionType;

int main()
{
    try
    {
        // Flecs 설정, Scene 수명, TransformSystem과 실제 Jolt 자세 왕복을 함께 확인하는 통합 fixture.
        flecs::world world;
        Scene pending(world);
        ExpectThrows<std::logic_error>([&] { pending.CreateEntity(); }, "inactive Scene cannot create Entities");
        grasplink::physics::PhysicsWorld physics;
        SceneManager scenes(world);
        scenes.LoadScene<Scene>();
        scenes.OnUpdate(0.0F);
        Scene& scene = *scenes.GetActiveScene();
        // 중간 anonymous Entity에는 TRS component가 없다. 그래도 조상의 World 변환은 자식에 누적되어야 한다.
        Entity ancestor = scene.CreateEntity("Ancestor");
        ancestor.SetLocalPosition({2.0F, 0.0F, 0.0F});
        auto group = world.entity().child_of(ancestor.GetHandle());
        Entity falling = scene.CreateEntity("Falling");
        falling.SetParent(Entity{group});
        Require(ancestor.FindChildByNameRecursive("Falling") == falling, "named lookup traverses anonymous grouping Entity");
        falling.SetLocalPosition({0.0F, 2.0F, 0.0F});
        falling.set<RigidBody>({BodyMotionType::Dynamic}).set<Colliders>({{physics_colliders::Box({0.1F, 0.1F, 0.1F})}});
        TransformSystemModule::UpdateWorldTransforms(world);
        RequireNear(falling.GetWorldMatrix()[3].x, 2.0, 1e-5, "ancestor through transform-less group");
        grasplink::simulation::PhysicsSystemModule integration(world, physics);
        auto step = [&]
        {
            // 매 4 ms fixed 간격마다 Local→World를 만들고, Dynamic 결과를 Local에 받은 뒤 World를 다시 계산한다.
            TransformSystemModule::UpdateWorldTransforms(world);
            integration.Step(0.004);
            TransformSystemModule::UpdateWorldTransforms(world);
        };
        // 짧게 낙하시켜 Jolt World 결과가 Entity Local로 돌아오는 좌표 방향을 확인한다.
        for (int i = 0; i < 30; ++i) step();
        Require(falling.GetLocalPosition().y < 1.99F, "Dynamic pose written to Local");
        RequireNear(falling.GetLocalPosition().x, 0.0, 1e-4, "group parent removed from Physics World pose");
        // Collider 제거는 Binding과 Jolt Body 연결도 끊어야 한다. 재추가하면 현재 pose에서 다시 낙하한다.
        falling.Remove<Colliders>();
        const float frozen = falling.GetLocalPosition().y;
        for (int i = 0; i < 30; ++i) step();
        RequireNear(falling.GetLocalPosition().y, frozen, 1e-6, "removed collider disconnects Dynamic feedback");
        falling.set<Colliders>({{physics_colliders::Box({0.1F, 0.1F, 0.1F})}});
        for (int i = 0; i < 30; ++i) step();
        Require(falling.GetLocalPosition().y < frozen - 0.01F, "reconfigured collider resumes Physics");

        // Dynamic 부모와 자식의 독립 이동을 계층 World pose에 합치는 규칙은 없으므로 자식 Body는 거부된다.
        Entity unsupported = scene.CreateEntity("UnsupportedChild");
        unsupported.SetLocalPosition({0.0F, 0.5F, 0.0F});
        unsupported.SetParent(falling);
        unsupported.set<RigidBody>({BodyMotionType::Dynamic}).set<Colliders>({{physics_colliders::Box({0.05F, 0.05F, 0.05F})}});
        for (int i = 0; i < 20; ++i) step();
        RequireNear(unsupported.GetLocalPosition().y, 0.5, 1e-6, "Dynamic ancestor hierarchy rejected");
        ExpectThrows<std::invalid_argument>([&] { falling.SetParent(Entity{world.entity("External")}); }, "cannot leave owning Scene");
        ExpectThrows<std::invalid_argument>([&] { falling.SetParent(unsupported); }, "hierarchy cycle rejected");

        // Static floor와 box의 half extent 합은 0.2 m다. 20 mm 허용 폭은 solver 오차를 허용하면서
        // 실제 접촉 높이가 맞는지 검증한다.
        Entity floor = scene.CreateEntity("Floor");
        floor.set<RigidBody>({BodyMotionType::Static, grasplink::physics::CollisionLayer::Environment})
            .set<Colliders>({{physics_colliders::Box({1.0F, 0.1F, 1.0F})}});
        Entity drop = scene.CreateEntity("Drop");
        drop.SetLocalPosition({0.0F, 1.0F, 0.0F});
        drop.set<RigidBody>({BodyMotionType::Dynamic}).set<Colliders>({{physics_colliders::Box({0.1F, 0.1F, 0.1F})}});
        // 500 × 4 ms = 2 s 동안 낙하와 접촉이 정착한다.
        for (int i = 0; i < 500; ++i) step();
        RequireNear(drop.GetLocalPosition().y, 0.2, 0.02, "ECS configured Static contact");
        floor.Destroy();
        // 접촉 중 Jolt가 Body를 sleep시킬 수 있다. Collider 재설정으로 새 Body를 만들어 wake 정책과
        // 무관하게 삭제된 floor가 더는 낙하를 막지 않는지 검사한다.
        drop.set<Colliders>({{physics_colliders::Box({0.1F, 0.1F, 0.1F})}});
        for (int i = 0; i < 150; ++i) step();
        Require(drop.GetLocalPosition().y < -0.2F, "Entity deletion removes Jolt Collider");

        // integration은 살아 있어도 이전 Scene의 hierarchy와 Binding이 파괴된 뒤 다음 Step이 안전해야 한다.
        const auto oldRoot = scene.GetSceneRoot();
        scenes.LoadScene<Scene>();
        scenes.OnUpdate(0.0F);
        Require(!oldRoot.is_alive() && !falling.IsValid() && !drop.IsValid(), "Scene transition destroys old hierarchy");
        step();
        std::cout << "Transform, Scene and Physics ECS lifetime checks passed\n";
        return 0;
    }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
