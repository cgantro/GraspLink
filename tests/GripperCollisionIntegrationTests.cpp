#include "Entity.h"
#include "PhysicsWorld.h"
#include "assets/GltfLoader.h"
#include "scene/SceneManager.h"
#include "simulation/components/PhysicsComponents.h"
#include "simulation/robotics/GripperColliders.h"
#include "simulation/systems/PhysicsSystemModule.h"
#include "systems/TransformSystemModule.h"
#include "TestSupport.h"

#include <glm/gtc/matrix_transform.hpp>

#include <filesystem>
#include <iostream>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace
{
using grasplink::physics::BodyMotionType;
using grasplink::physics::CollisionLayer;

struct Fixture
{
    flecs::world world;
    SceneManager scenes{world};
    grasplink::physics::PhysicsWorld physics;
    Scene* scene = nullptr;
    ModelResource model;
    Entity robotRoot;
    Entity gripper;

    Fixture()
    {
        scenes.LoadScene<Scene>();
        scenes.OnUpdate(0.0F);
        scene = scenes.GetActiveScene();
        Require(scene != nullptr, "active Scene created");
        model = GltfLoader::LoadGLB(std::filesystem::path("assets") / "HCR12A_2F-85.glb");
        robotRoot = scene->CreateEntity("RobotRoot");
        const int gripperIndex = FindNode("Gripper");
        Require(gripperIndex >= 0, "GLB has Gripper frame");
        gripper = CreateSubtree(gripperIndex, robotRoot);
    }

    int FindNode(const std::string& name) const
    {
        for (std::size_t i = 0; i < model.nodes.size(); ++i)
            if (model.nodes[i].name == name) return static_cast<int>(i);
        return -1;
    }

    Entity CreateSubtree(int nodeIndex, const Entity& parent)
    {
        const NodeData& node = model.nodes.at(static_cast<std::size_t>(nodeIndex));
        Entity entity = scene->CreateEntity(node.name);
        entity.SetParent(parent);
        entity.SetLocalPosition(node.translation);
        entity.SetLocalRotation(node.rotation);
        entity.SetLocalScale(node.scale);
        for (int child : node.childrenIndices) CreateSubtree(child, entity);
        return entity;
    }

    std::size_t CountDescendants(const Entity& entity) const
    {
        std::size_t count = 0;
        for (const Entity& child : entity.GetChildren())
            count += 1 + CountDescendants(child);
        return count;
    }

    void Step(int count = 1)
    {
        TransformSystemModule::UpdateWorldTransforms(world);
        for (int i = 0; i < count; ++i)
        {
            integration->Step(0.004);
            TransformSystemModule::UpdateWorldTransforms(world);
        }
    }

    std::unique_ptr<grasplink::simulation::PhysicsSystemModule> integration;
};

glm::vec3 WorldPoint(const Entity& entity, const glm::vec3& local)
{
    return glm::vec3(entity.GetWorldMatrix() * glm::vec4(local, 1.0F));
}

glm::vec3 TopSurfacePoint(Entity proxy, float& topY)
{
    std::vector<glm::vec3> points;
    for (const auto& shape : proxy.Get<Colliders>().shapes)
        for (const auto& point : shape.pointsMeters)
            points.push_back(WorldPoint(proxy, point));
    topY = -100.0F;
    for (const auto& point : points) topY = std::max(topY, point.y);
    glm::vec3 center(0.0F);
    std::size_t count = 0;
    for (const auto& point : points)
        if (topY - point.y < 0.002F)
        {
            center += point;
            ++count;
        }
    Require(count != 0, "tip hull has a gravity-facing support point");
    return center / static_cast<float>(count);
}

void DropOnTip(Fixture& fixture, const char* tipJoint, const char* proxyName)
{
    Entity proxy = fixture.gripper.FindChildByNameRecursive(proxyName);
    Require(proxy.IsValid(), std::string(proxyName) + " exists");
    const auto shapes = proxy.Get<Colliders>().shapes;
    Require(!shapes.empty(), std::string(proxyName) + " has collider geometry");
    const Entity joint = fixture.gripper.FindChildByNameRecursive(tipJoint);
    Require(joint.IsValid(), std::string(tipJoint) + " exists");
    float surfaceY = 0.0F;
    const glm::vec3 surface = TopSurfacePoint(proxy, surfaceY);

    Entity drop = fixture.scene->CreateEntity(std::string("Drop_") + tipJoint);
    drop.SetLocalPosition({surface.x, surfaceY + 0.07F, surface.z});
    drop.set<RigidBody>({BodyMotionType::Dynamic, CollisionLayer::DynamicObject})
        .set<Colliders>({{physics_colliders::Box({0.004F, 0.004F, 0.004F})}});
    fixture.Step(300);
    Require(drop.GetLocalPosition().y > surfaceY + 0.002F && drop.GetLocalPosition().y < surfaceY + 0.02F,
        std::string("dynamic object contacts ") + tipJoint + " collider");
}
}

int main()
{
    try
    {
        Fixture fixture;
        const std::size_t beforeInvalid = fixture.CountDescendants(fixture.robotRoot);
        ModelResource missingMesh = fixture.model;
        for (NodeData& node : missingMesh.nodes)
            if (node.name == "LeftFingerTipMesh") node.name = "MissingFingerTipMesh";
        ExpectThrows<std::invalid_argument>([&]
        {
            grasplink::simulation::ConfigureTwoF85Colliders(*fixture.scene, fixture.robotRoot, missingMesh);
        }, "missing required mesh rejected");
        Require(fixture.CountDescendants(fixture.robotRoot) == beforeInvalid,
            "missing mesh is rejected before creating proxy entities");
        Entity misplacedMesh = fixture.gripper.FindChildByNameRecursive("LeftFingerTipMesh");
        const Entity originalMeshParent = misplacedMesh.GetParent();
        misplacedMesh.SetParent(fixture.robotRoot);
        ExpectThrows<std::invalid_argument>([&]
        {
            grasplink::simulation::ConfigureTwoF85Colliders(*fixture.scene, fixture.robotRoot, fixture.model);
        }, "mesh outside its owning joint hierarchy rejected");
        Require(fixture.CountDescendants(fixture.robotRoot) == beforeInvalid,
            "wrong hierarchy is rejected before creating proxy entities");
        misplacedMesh.SetParent(originalMeshParent);

        grasplink::simulation::ConfigureTwoF85Colliders(*fixture.scene, fixture.robotRoot, fixture.model);
        TransformSystemModule::UpdateWorldTransforms(fixture.world);

        const std::vector<std::string> proxies{
            "Gripper_CollisionProxy", "LeftOuterKnuckleJoint_CollisionProxy", "LeftInnerKnuckleJoint_CollisionProxy",
            "LeftFingerTipJoint_CollisionProxy", "RightOuterKnuckleJoint_CollisionProxy", "RightInnerKnuckleJoint_CollisionProxy",
            "RightFingerTipJoint_CollisionProxy"};
        for (const auto& name : proxies)
        {
            Entity proxy = fixture.gripper.FindChildByNameRecursive(name);
            Require(proxy.IsValid(), name + " exists");
            Require(glm::length(proxy.GetLocalPosition()) < 1e-6F, name + " identity local translation");
            Require(glm::length(proxy.GetLocalRotation()) < 1e-6F, name + " identity local rotation");
            Require(glm::length(proxy.GetLocalScale() - glm::vec3(1.0F)) < 1e-6F, name + " identity local scale");
            Require(proxy.Get<RigidBody>().motionType == BodyMotionType::Kinematic, name + " is Kinematic");
            Require(proxy.Get<RigidBody>().collisionLayer == CollisionLayer::Gripper, name + " uses Gripper layer");
            Require(!proxy.Get<Colliders>().shapes.empty(), name + " has own hulls");
            for (const auto& shape : proxy.Get<Colliders>().shapes)
                Require(shape.type == grasplink::physics::CollisionShapeType::ConvexHull && shape.pointsMeters.size() >= 4,
                    name + " consists of convex support point hulls");
        }

        Entity leftTip = fixture.gripper.FindChildByNameRecursive("LeftFingerTipJoint_CollisionProxy");
        Entity rightTip = fixture.gripper.FindChildByNameRecursive("RightFingerTipJoint_CollisionProxy");
        const glm::mat4 worldToGripper = glm::inverse(fixture.gripper.GetWorldMatrix());
        float leftMaxX = -1.0F;
        float rightMinX = 1.0F;
        for (const auto& shape : leftTip.Get<Colliders>().shapes)
            for (const auto& point : shape.pointsMeters)
            {
                const glm::vec3 inFrame(worldToGripper * glm::vec4(WorldPoint(leftTip, point), 1.0F));
                leftMaxX = std::max(leftMaxX, inFrame.x);
            }
        for (const auto& shape : rightTip.Get<Colliders>().shapes)
            for (const auto& point : shape.pointsMeters)
            {
                const glm::vec3 inFrame(worldToGripper * glm::vec4(WorldPoint(rightTip, point), 1.0F));
                rightMinX = std::min(rightMinX, inFrame.x);
            }
        RequireNear(rightMinX - leftMaxX, 0.085, 1.0e-5, "open fingertip aperture is 85 mm");

        fixture.robotRoot.SetLocalPosition({0.0F, 0.3F, 0.0F});
        fixture.gripper.SetLocalPosition(glm::vec3(0.0F));
        fixture.gripper.SetLocalRotation({1.5707963F, 0.0F, 0.0F});
        TransformSystemModule::UpdateWorldTransforms(fixture.world);
        Require(fixture.CountDescendants(fixture.robotRoot) == beforeInvalid + 7,
            "successful configuration creates exactly seven proxies");
        fixture.integration = std::make_unique<grasplink::simulation::PhysicsSystemModule>(fixture.world, fixture.physics);
        TransformSystemModule::UpdateWorldTransforms(fixture.world);

        DropOnTip(fixture, "LeftFingerTipJoint", "LeftFingerTipJoint_CollisionProxy");
        DropOnTip(fixture, "RightFingerTipJoint", "RightFingerTipJoint_CollisionProxy");

        Entity baseProxy = fixture.gripper.FindChildByNameRecursive("Gripper_CollisionProxy");
        float baseSurfaceY = 0.0F;
        const glm::vec3 baseSurface = TopSurfacePoint(baseProxy, baseSurfaceY);
        std::vector<std::pair<Entity, Colliders>> otherProxyColliders;
        for (const auto& name : proxies)
        {
            if (name == "Gripper_CollisionProxy") continue;
            Entity proxy = fixture.gripper.FindChildByNameRecursive(name);
            otherProxyColliders.emplace_back(proxy, proxy.Get<Colliders>());
            proxy.Remove<Colliders>();
        }
        Entity baseDrop = fixture.scene->CreateEntity("BaseBodyDrop");
        baseDrop.SetLocalPosition({baseSurface.x, baseSurfaceY + 0.07F, baseSurface.z});
        baseDrop.set<RigidBody>({BodyMotionType::Dynamic, CollisionLayer::DynamicObject})
            .set<Colliders>({{physics_colliders::Box({0.004F, 0.004F, 0.004F})}});
        fixture.Step(300);
        Require(baseDrop.GetLocalPosition().y > baseSurfaceY + 0.002F &&
                baseDrop.GetLocalPosition().y < baseSurfaceY + 0.02F,
            "dynamic object contacts fixed Gripper body collider: body y=" +
                std::to_string(baseDrop.GetLocalPosition().y) + ", surface y=" + std::to_string(baseSurfaceY));
        for (auto& [proxy, colliders] : otherProxyColliders) proxy.set<Colliders>(colliders);
        fixture.Step(1);

        Entity apertureDrop = fixture.scene->CreateEntity("OpenApertureDrop");
        apertureDrop.SetLocalPosition({0.0F, 0.7F, 0.13F});
        apertureDrop.set<RigidBody>({BodyMotionType::Dynamic, CollisionLayer::DynamicObject})
            .set<Colliders>({{physics_colliders::Box({0.003F, 0.003F, 0.003F})}});
        fixture.Step(350);
        Require(apertureDrop.GetLocalPosition().y < -0.15F, "dynamic object passes through open finger aperture");

        float oldSurfaceY = 0.0F;
        const glm::vec3 oldSurface = TopSurfacePoint(leftTip, oldSurfaceY);
        fixture.robotRoot.SetLocalPosition({0.0F, 0.6F, 0.0F});
        fixture.Step(2);
        float newSurfaceY = 0.0F;
        const glm::vec3 newSurface = TopSurfacePoint(leftTip, newSurfaceY);
        Require(newSurfaceY > oldSurfaceY + 0.25F, "moving Gripper frame moves Kinematic collider pose");
        Entity movedRootContact = fixture.scene->CreateEntity("MovedRootContact");
        movedRootContact.SetLocalPosition({newSurface.x, newSurfaceY + 0.07F, newSurface.z});
        movedRootContact.set<RigidBody>({BodyMotionType::Dynamic, CollisionLayer::DynamicObject})
            .set<Colliders>({{physics_colliders::Box({0.003F, 0.003F, 0.003F})}});
        fixture.Step(300);
        Require(movedRootContact.GetLocalPosition().y > newSurfaceY + 0.002F &&
                movedRootContact.GetLocalPosition().y < newSurfaceY + 0.02F,
            "dynamic object contacts fingertip after root collider moves");
        Entity rootMoveDrop = fixture.scene->CreateEntity("RootMovedDrop");
        rootMoveDrop.SetLocalPosition({oldSurface.x, oldSurfaceY + 0.07F, oldSurface.z});
        rootMoveDrop.set<RigidBody>({BodyMotionType::Dynamic, CollisionLayer::DynamicObject})
            .set<Colliders>({{physics_colliders::Box({0.003F, 0.003F, 0.003F})}});
        fixture.Step(280);
        Require(rootMoveDrop.GetLocalPosition().y < oldSurfaceY - 0.05F,
            "object at old fingertip position falls after root collider moves");

        Entity leftJoint = fixture.gripper.FindChildByNameRecursive("LeftFingerTipJoint");
        float oldFingerSurfaceY = 0.0F;
        const glm::vec3 oldFingerSurface = TopSurfacePoint(leftTip, oldFingerSurfaceY);
        leftJoint.SetLocalRotation(leftJoint.GetLocalRotation() + glm::vec3{0.0F, 0.0F, 0.45F});
        fixture.Step(2);
        float newFingerSurfaceY = 0.0F;
        const glm::vec3 newFingerSurface = TopSurfacePoint(leftTip, newFingerSurfaceY);
        Require(glm::length(newFingerSurface - oldFingerSurface) > 0.01F,
            "finger joint rotation moves its Kinematic proxy surface");
        Entity movedFingerDrop = fixture.scene->CreateEntity("MovedFingerDrop");
        movedFingerDrop.SetLocalPosition({newFingerSurface.x, newFingerSurfaceY + 0.07F, newFingerSurface.z});
        movedFingerDrop.set<RigidBody>({BodyMotionType::Dynamic, CollisionLayer::DynamicObject})
            .set<Colliders>({{physics_colliders::Box({0.003F, 0.003F, 0.003F})}});
        fixture.Step(280);
        Require(movedFingerDrop.GetLocalPosition().y > newFingerSurfaceY + 0.002F &&
                movedFingerDrop.GetLocalPosition().y < newFingerSurfaceY + 0.02F,
            "object contacts the physically rotated fingertip collider");

        ExpectThrows<std::invalid_argument>([&]
        {
            grasplink::simulation::ConfigureTwoF85Colliders(*fixture.scene, fixture.robotRoot, fixture.model);
        }, "repeated configuration rejected");
        Require(fixture.CountDescendants(fixture.robotRoot) == beforeInvalid + 7,
            "repeated configuration does not create partial proxies");

        const Entity oldSceneRoot(fixture.scene->GetSceneRoot());
        const Entity oldProxy = fixture.gripper.FindChildByNameRecursive("Gripper_CollisionProxy");
        fixture.scenes.LoadScene<Scene>();
        fixture.scenes.OnUpdate(0.0F);
        Require(!oldSceneRoot.GetHandle().is_alive() && !oldProxy.IsValid(), "Scene cleanup removes proxy hierarchy");

        std::cout << "TwoF-85 collider hierarchy and contact checks passed\n";
        return 0;
    }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
