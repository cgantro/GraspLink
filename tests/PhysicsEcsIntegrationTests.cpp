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

/**
 * @brief 각 Body Motion Type의 Scene/Physics 자세 권위를 확인한다.
 * @details Static과 Kinematic은 Scene이 목표를 작성하고 Dynamic은 Jolt 계산 결과를 Scene으로 받는다.
 */
void CheckPoseAuthority()
{
    using grasplink::physics::IsPhysicsDriven;
    using grasplink::physics::IsSceneDriven;
    Require(IsSceneDriven(BodyMotionType::Static) && !IsPhysicsDriven(BodyMotionType::Static),
        "Static pose is Scene driven");
    Require(IsSceneDriven(BodyMotionType::Kinematic) && !IsPhysicsDriven(BodyMotionType::Kinematic),
        "Kinematic pose is Scene driven");
    Require(!IsSceneDriven(BodyMotionType::Dynamic) && IsPhysicsDriven(BodyMotionType::Dynamic),
        "Dynamic pose is Physics driven");
}

/**
 * @brief Scene/Physics 동기화, 부모 계층 제한과 실제 접촉 자세를 확인한다.
 * @details Dynamic은 항등 grouping 계층에서만 물리 결과를 받는다. Static/Kinematic은 Scene의 이동과
 * 회전 부모를 따라가며, Kinematic 이동 정착 후에는 잔류 속도 없이 같은 접촉 높이를 유지해야 한다.
 * Collider 재설정·삭제와 Scene 전환 시 Body 연결 수명도 함께 검사한다.
 */
int main()
{
    try
    {
        CheckPoseAuthority();
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
        Entity blocked = scene.CreateEntity("BlockedDynamic");
        blocked.SetParent(Entity{group});
        Require(ancestor.FindChildByNameRecursive("BlockedDynamic") == blocked, "named lookup traverses anonymous grouping Entity");
        blocked.SetLocalPosition({0.0F, 2.0F, 0.0F});
        blocked.set<RigidBody>({BodyMotionType::Dynamic}).set<Colliders>({{physics_colliders::Box({0.1F, 0.1F, 0.1F})}});
        auto identityGroup = world.entity().child_of(scene.GetSceneRoot());
        Entity falling = scene.CreateEntity("Falling");
        falling.SetParent(Entity{identityGroup});
        // 별도 접촉 fixture와 떨어진 x=-5 m에서 항등 grouping 부모의 Dynamic 낙하를 검사한다.
        falling.SetLocalPosition({-5.0F, 2.0F, 0.0F});
        falling.set<RigidBody>({BodyMotionType::Dynamic}).set<Colliders>({{physics_colliders::Box({0.1F, 0.1F, 0.1F})}});
        TransformSystemModule::UpdateWorldTransforms(world);
        RequireNear(blocked.GetWorldMatrix()[3].x, 2.0, 1e-5, "ancestor through transform-less group");
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
        RequireNear(blocked.GetLocalPosition().y, 2.0, 1e-6, "Dynamic with translated ancestor hierarchy rejected");
        // 다음 부모 변경 검증이 조상을 항등으로 복구해도 이 독립 negative fixture는 접촉 계산에 참여하지 않는다.
        blocked.Remove<Colliders>();
        Require(falling.GetLocalPosition().y < 1.99F, "Dynamic pose written to Local");
        RequireNear(falling.GetLocalPosition().x, -5.0, 1e-4, "identity group preserves Dynamic World x");
        // Collider 제거는 Binding과 Jolt Body 연결도 끊어야 한다. 재추가하면 현재 pose에서 다시 낙하한다.
        falling.Remove<Colliders>();
        const float frozen = falling.GetLocalPosition().y;
        for (int i = 0; i < 30; ++i) step();
        RequireNear(falling.GetLocalPosition().y, frozen, 1e-6, "removed collider disconnects Dynamic feedback");
        falling.set<Colliders>({{physics_colliders::Box({0.1F, 0.1F, 0.1F})}});
        for (int i = 0; i < 30; ++i) step();
        Require(falling.GetLocalPosition().y < frozen - 0.01F, "reconfigured collider resumes Physics");

        // 이미 생성된 Body도 부모나 조상의 TRS가 바뀌면 제거된다. 거부된 동안 Local pose가 권위값으로 남는다.
        falling.SetParent(Entity{group});
        falling.SetLocalPosition({-5.0F, 3.0F, 0.0F});
        for (int i = 0; i < 20; ++i) step();
        RequireNear(falling.GetLocalPosition().y, 3.0, 1e-6, "runtime translated parent disconnects Dynamic feedback");
        ancestor.SetLocalPosition({0.0F, 0.0F, 0.0F});
        step();
        Require(falling.GetLocalPosition().y < 3.0F, "identity ancestor recreates Dynamic Body");
        ancestor.SetLocalRotation(glm::angleAxis(0.4F, glm::vec3{0.0F, 1.0F, 0.0F}));
        falling.SetLocalPosition({-5.0F, 3.0F, 0.0F});
        for (int i = 0; i < 20; ++i) step();
        RequireNear(falling.GetLocalPosition().y, 3.0, 1e-6, "runtime rotated parent disconnects Dynamic feedback");
        ancestor.SetLocalRotation(glm::quat{1.0F, 0.0F, 0.0F, 0.0F});
        step();
        Require(falling.GetLocalPosition().y < 3.0F, "identity rotation recreates Dynamic Body");
        ancestor.SetLocalScale({2.0F, 2.0F, 2.0F});
        falling.SetLocalPosition({-5.0F, 3.0F, 0.0F});
        for (int i = 0; i < 20; ++i) step();
        RequireNear(falling.GetLocalPosition().y, 3.0, 1e-6, "runtime scaled parent disconnects Dynamic feedback");
        ancestor.SetLocalScale({1.0F, 1.0F, 1.0F});
        falling.SetParent(Entity{identityGroup});
        for (int i = 0; i < 30; ++i) step();
        Require(falling.GetLocalPosition().y < 2.99F && falling.GetLocalPosition().y > 2.8F,
            "supported parent recreates Dynamic Body from current Scene pose");

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
        Entity floorParent = scene.CreateEntity("FloorParent");
        floorParent.SetLocalRotation(glm::angleAxis(0.3F, glm::vec3{0.0F, 1.0F, 0.0F}));
        Entity floor = scene.CreateEntity("Floor");
        floor.SetParent(floorParent);
        floor.set<RigidBody>({BodyMotionType::Static, grasplink::physics::CollisionLayer::Environment})
            .set<Colliders>({{physics_colliders::Box({1.0F, 0.1F, 1.0F})}});
        Entity drop = scene.CreateEntity("Drop");
        drop.SetLocalPosition({0.0F, 1.0F, 0.0F});
        drop.set<RigidBody>({BodyMotionType::Dynamic}).set<Colliders>({{physics_colliders::Box({0.1F, 0.1F, 0.1F})}});
        // 500 × 4 ms = 2 s 동안 낙하와 접촉이 정착한다.
        for (int i = 0; i < 500; ++i) step();
        RequireNear(drop.GetLocalPosition().y, 0.2, 0.02, "ECS configured Static contact");
        // Static은 Scene에서 옮긴 World pose를 계산 전에 반영해야 새 높이에서 접촉한다.
        floorParent.SetLocalPosition({0.0F, 0.4F, 0.0F});
        drop.SetLocalPosition({0.0F, 1.4F, 0.0F});
        drop.set<Colliders>({{physics_colliders::Box({0.1F, 0.1F, 0.1F})}});
        for (int i = 0; i < 500; ++i) step();
        RequireNear(drop.GetLocalPosition().y, 0.6, 0.02, "moved Static parent updates physical contact height");
        floor.Destroy();
        // 접촉 중 Jolt가 Body를 sleep시킬 수 있다. Collider 재설정으로 새 Body를 만들어 wake 정책과
        // 무관하게 삭제된 floor가 더는 낙하를 막지 않는지 검사한다.
        drop.set<Colliders>({{physics_colliders::Box({0.1F, 0.1F, 0.1F})}});
        for (int i = 0; i < 150; ++i) step();
        Require(drop.GetLocalPosition().y < -0.2F, "Entity deletion removes Jolt Collider");

        Entity platformParent = scene.CreateEntity("PlatformParent");
        platformParent.SetLocalPosition({5.0F, 0.0F, 0.0F});
        platformParent.SetLocalRotation(glm::angleAxis(-0.4F, glm::vec3{0.0F, 1.0F, 0.0F}));
        Entity platform = scene.CreateEntity("KinematicPlatform");
        platform.SetParent(platformParent);
        platform.set<RigidBody>({BodyMotionType::Kinematic, grasplink::physics::CollisionLayer::Environment})
            .set<Colliders>({{physics_colliders::Box({1.0F, 0.1F, 1.0F})}});
        step();
        platform.SetLocalPosition({0.0F, 0.4F, 0.0F});
        platform.SetLocalRotation(glm::angleAxis(0.2F, glm::vec3{0.0F, 1.0F, 0.0F}));
        step();
        // 이동 목표에 도달한 다음 간격은 속도를 0으로 정착시킨다. 이후 같은 목표는 재이동할 필요가 없다.
        step();
        Entity platformDrop = scene.CreateEntity("PlatformDrop");
        platformDrop.SetLocalPosition({5.3F, 1.4F, 0.0F});
        platformDrop.set<RigidBody>({BodyMotionType::Dynamic})
            .set<Colliders>({{physics_colliders::Box({0.1F, 0.1F, 0.1F})}});
        for (int i = 0; i < 500; ++i) step();
        RequireNear(platformDrop.GetLocalPosition().y, 0.6, 0.02, "Kinematic transformed parent sets physical contact height");
        const glm::vec3 settled = platformDrop.GetLocalPosition();
        for (int i = 0; i < 250; ++i) step();
        RequireNear(platformDrop.GetLocalPosition().y, settled.y, 0.005, "unchanged Kinematic target has no vertical contact drift");
        RequireNear(platformDrop.GetLocalPosition().x, settled.x, 0.005, "unchanged Kinematic target has no horizontal contact drift");
        RequireNear(platformDrop.GetLocalPosition().z, settled.z, 0.005, "unchanged Kinematic target has no depth contact drift");

        // integration은 살아 있어도 이전 Scene의 hierarchy와 Binding이 파괴된 뒤 다음 Step이 안전해야 한다.
        const auto oldRoot = scene.GetSceneRoot();
        scenes.LoadScene<Scene>();
        scenes.OnUpdate(0.0F);
        Require(!oldRoot.is_alive() && !falling.IsValid() && !drop.IsValid(), "Scene transition destroys old hierarchy");
        step();
        std::cout << "Transform, Scene, Physics authority, parent, contact and lifetime checks passed\n";
        return 0;
    }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
