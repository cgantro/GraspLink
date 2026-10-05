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
        flecs::world world;
        Scene pending(world);
        ExpectThrows<std::logic_error>([&] { pending.CreateEntity(); }, "inactive Scene cannot create Entities");
        grasplink::physics::PhysicsWorld physics;
        SceneManager scenes(world);
        scenes.LoadScene<Scene>();
        scenes.OnUpdate(0.0F);
        Scene& scene = *scenes.GetActiveScene();
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
            TransformSystemModule::UpdateWorldTransforms(world);
            integration.Step(0.004);
            TransformSystemModule::UpdateWorldTransforms(world);
        };
        for (int i = 0; i < 30; ++i) step();
        Require(falling.GetLocalPosition().y < 1.99F, "Dynamic pose written to Local");
        RequireNear(falling.GetLocalPosition().x, 0.0, 1e-4, "group parent removed from Physics World pose");
        falling.Remove<Colliders>();
        const float frozen = falling.GetLocalPosition().y;
        for (int i = 0; i < 30; ++i) step();
        RequireNear(falling.GetLocalPosition().y, frozen, 1e-6, "removed collider disconnects Dynamic feedback");
        falling.set<Colliders>({{physics_colliders::Box({0.1F, 0.1F, 0.1F})}});
        for (int i = 0; i < 30; ++i) step();
        Require(falling.GetLocalPosition().y < frozen - 0.01F, "reconfigured collider resumes Physics");

        Entity unsupported = scene.CreateEntity("UnsupportedChild");
        unsupported.SetLocalPosition({0.0F, 0.5F, 0.0F});
        unsupported.SetParent(falling);
        unsupported.set<RigidBody>({BodyMotionType::Dynamic}).set<Colliders>({{physics_colliders::Box({0.05F, 0.05F, 0.05F})}});
        for (int i = 0; i < 20; ++i) step();
        RequireNear(unsupported.GetLocalPosition().y, 0.5, 1e-6, "Dynamic ancestor hierarchy rejected");
        ExpectThrows<std::invalid_argument>([&] { falling.SetParent(Entity{world.entity("External")}); }, "cannot leave owning Scene");
        ExpectThrows<std::invalid_argument>([&] { falling.SetParent(unsupported); }, "hierarchy cycle rejected");

        // 삭제된 Static Collider가 실제 접촉에서도 사라지는지 확인한다.
        Entity floor = scene.CreateEntity("Floor");
        floor.set<RigidBody>({BodyMotionType::Static, grasplink::physics::CollisionLayer::Environment})
            .set<Colliders>({{physics_colliders::Box({1.0F, 0.1F, 1.0F})}});
        Entity drop = scene.CreateEntity("Drop");
        drop.SetLocalPosition({0.0F, 1.0F, 0.0F});
        drop.set<RigidBody>({BodyMotionType::Dynamic}).set<Colliders>({{physics_colliders::Box({0.1F, 0.1F, 0.1F})}});
        for (int i = 0; i < 500; ++i) step();
        RequireNear(drop.GetLocalPosition().y, 0.2, 0.02, "ECS configured Static contact");
        floor.Destroy();
        // 접촉 물체의 sleep 정책에 기대지 않고 Dynamic Body를 다시 활성 상태로 구성한다.
        drop.set<Colliders>({{physics_colliders::Box({0.1F, 0.1F, 0.1F})}});
        for (int i = 0; i < 150; ++i) step();
        Require(drop.GetLocalPosition().y < -0.2F, "Entity deletion removes Jolt Collider");

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
