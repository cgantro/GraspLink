#include "PhysicsWorld.h"
#include "TestSupport.h"

#include <glm/gtc/quaternion.hpp>
#include <iostream>
#include <limits>

using namespace grasplink::physics;

namespace
{
/**
 * @brief Body 원점과 위치·회전이 따로 있는 바닥 Box fixture.
 * @details Body 회전은 World Z축 기준 +90°다. Body 원점에서 Local X=0.5 m에 있는 형상 중심은
 * 회전 뒤 World Y=0.5 m가 되고, Local X 반쪽 길이 0.25 m가 World 수직 반높이가 된다.
 * 따라서 Body 원점은 Y=0에 남고 플랫폼 윗면은 Y=0.75 m다. 위치와 방향이 원점과 다른
 * 형상을 써야 COM을 Body 원점으로 잘못 반환하는 회귀를 접촉 위치로 드러낼 수 있다.
 */
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

/**
 * @brief 지정한 X 위치의 구가 offset 플랫폼에 안착하는지 검증한다.
 * @param world 테스트용 PhysicsWorld. 호출자가 플랫폼을 만들고 수명을 관리한다.
 * @param x 플랫폼과 구의 World X 위치 [m].
 * @param expectedY 구 Body 원점의 기대 높이 [m].
 * @details 플랫폼 윗면 0.75 m에 반지름 0.1 m인 구가 닿으므로 구 중심은 0.85 m다.
 * 750회의 4 ms step은 3 s 동안 낙하·접촉·안정화할 시간을 준다. 15 mm 허용치는 solver의
 * 작은 잔류 오차를 허용하면서 Body 원점/COM 혼동으로 생기는 큰 높이 오차는 잡는다.
 */
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
    RequireNear(world.GetBodyTransform(falling).position.y, expectedY, 0.015, "offset platform contact height");
    world.DestroyBody(falling);
}
}

int main()
{
    try
    {
        // 생성·접촉·Teleport 후 Body 원점을 확인하고, 동일 접촉 높이를 두 X 위치에서 재검증한다.
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

        // 핸들은 World별 token과 Body ID를 함께 식별한다. 파괴 뒤 ID가 재사용돼도 이전 handle은 무효다.
        PhysicsWorld otherWorld;
        const auto other = otherWorld.CreateBody(OffsetPlatform());
        Require(otherWorld.IsBodyValid(other), "own World handle");
        Require(!otherWorld.IsBodyValid(handle), "cross-World handle must fail");
        ExpectThrows<std::invalid_argument>([&] { otherWorld.GetBodyTransform(handle); }, "cross-World getter must fail");
        world.DestroyBody(handle);
        Require(!world.IsBodyValid(handle), "destroyed handle must fail");
        const auto replacement = world.CreateBody(OffsetPlatform());
        Require(world.IsBodyValid(replacement) && !world.IsBodyValid(handle), "reused ID must not revive old handle");

        // Jolt 경계에 넘기기 전에 수치·회전 입력을 검증해 예측 가능한 예외를 낸다.
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
