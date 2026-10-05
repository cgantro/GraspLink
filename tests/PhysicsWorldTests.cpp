#include "PhysicsWorld.h"
#include "TestSupport.h"

#include <glm/gtc/quaternion.hpp>
#include <iostream>
#include <limits>

using namespace grasplink::physics;

namespace
{
BodyDescription OffsetPlatform()
{
    BodyDescription body;
    body.motionType = BodyMotionType::Static;
    body.collisionLayer = CollisionLayer::Environment;
    body.transform.rotation = glm::angleAxis(glm::radians(90.0F), glm::vec3{0.0F, 0.0F, 1.0F});
    CollisionShapeDescription box;
    box.halfExtentsMeters = {0.25F, 0.5F, 0.5F};
    box.localTransform.position = {0.5F, 0.0F, 0.0F};
    body.shapes.push_back(box);
    return body;
}

void CheckContact(PhysicsWorld& world, float x, float expectedY)
{
    BodyDescription sphere;
    CollisionShapeDescription shape;
    shape.type = CollisionShapeType::Sphere;
    shape.radiusMeters = 0.1F;
    sphere.shapes.push_back(shape);
    sphere.transform.position = {x, 2.0F, 0.0F};
    const auto falling = world.CreateBody(sphere);
    for (int i = 0; i < 750; ++i) world.Step(0.004);
    // 독립 기준: 플랫폼 윗면 0.75 m + 구 반지름 0.1 m. 원점 왕복만으로는 COM 중복을 찾지 못한다.
    RequireNear(world.GetBodyTransform(falling).position.y, expectedY, 0.015, "offset platform contact height");
    world.DestroyBody(falling);
}
}

int main()
{
    try
    {
        PhysicsWorld world;
        auto platform = OffsetPlatform();
        const auto handle = world.CreateBody(platform);
        RequireNear(world.GetBodyTransform(handle).position.y, 0.0, 1e-5, "body origin after creation");
        CheckContact(world, 0.0F, 0.85F);
        platform.transform.position.x = 1.5F;
        world.SetBodyTransform(handle, platform.transform);
        RequireNear(world.GetBodyTransform(handle).position.x, 1.5, 1e-5, "body origin after teleport");
        CheckContact(world, 1.5F, 0.85F);

        platform.motionType = BodyMotionType::Kinematic;
        platform.collisionLayer = CollisionLayer::Robot;
        platform.transform.position = {3.0F, 0.0F, 0.0F};
        const auto moving = world.CreateBody(platform);
        auto target = platform.transform;
        target.position.y = 0.25F;
        world.MoveKinematic(moving, target, 0.004);
        world.Step(0.004);
        RequireNear(world.GetBodyTransform(moving).position.y, 0.25, 1e-4, "kinematic body origin");
        world.DestroyBody(moving);

        PhysicsWorld otherWorld;
        const auto other = otherWorld.CreateBody(OffsetPlatform());
        Require(otherWorld.IsBodyValid(other), "own World handle");
        Require(!otherWorld.IsBodyValid(handle), "cross-World handle must fail");
        ExpectThrows<std::invalid_argument>([&] { otherWorld.GetBodyTransform(handle); }, "cross-World getter must fail");
        world.DestroyBody(handle);
        Require(!world.IsBodyValid(handle), "destroyed handle must fail");
        const auto replacement = world.CreateBody(OffsetPlatform());
        Require(world.IsBodyValid(replacement) && !world.IsBodyValid(handle), "reused ID must not revive old handle");

        auto invalid = OffsetPlatform();
        invalid.transform.position.x = std::numeric_limits<float>::quiet_NaN();
        ExpectThrows<std::invalid_argument>([&] { world.CreateBody(invalid); }, "non-finite pose must fail");
        invalid = OffsetPlatform();
        invalid.shapes.front().localTransform.rotation = glm::quat{0.0F, 0.0F, 0.0F, 0.0F};
        ExpectThrows<std::invalid_argument>([&] { world.CreateBody(invalid); }, "zero shape rotation must fail");
        std::cout << "Physics origin, contact, movement and ownership checks passed\n";
        return 0;
    }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
