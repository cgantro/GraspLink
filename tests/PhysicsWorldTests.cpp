#include "PhysicsWorld.h"
#include "TestSupport.h"

#include <glm/gtc/quaternion.hpp>
#include <iostream>
#include <limits>

using namespace grasplink::physics;

namespace
{
/**
 * @brief 물리 물체 기준점과 실제 바닥 상자 중심이 다른 플랫폼 시험 자료를 만든다.
 * @details Body는 Jolt에서 위치와 회전을 가진 물체다. 이 플랫폼의 상자 중심은 Body 기준점에서 Local X로 0.5 m 떨어져 있고, Z축으로 90도 돌리면 그 차이는 Scene Y 방향이 된다.
 * 상자 반높이 0.25 m까지 더하면 윗면은 Y=0.75 m에 있어야 한다. 구가 여기에 안착하는 높이를 확인하면 계산이 실제 형상 중심 대신 Body 기준점을 사용한 오류를 찾을 수 있다.
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
 * @details 플랫폼 윗면 0.75 m에 반지름 0.1 m인 구가 닿으면 구 중심은 0.85 m가 된다.
 * 750회의 4 ms 계산은 3 s 동안 낙하하고 접촉한 뒤 안정될 시간을 준다. 15 mm 허용 오차는 작은 계산 흔들림은 허용하면서 구 중심과 물체 기준점을 혼동한 큰 높이 오류를 잡는다.
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
        // Body를 만들고 다른 물체와 접촉시킨 뒤 순간이동시켜 원점이 올바른지 확인한다. 서로 다른 두 X 위치에서도 같은 접촉 높이가 유지되는지 검사한다.
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
        target.rotation = glm::angleAxis(0.4F, glm::vec3{0.0F, 1.0F, 0.0F}) * target.rotation;
        world.MoveKinematic(moving, target, 0.004);
        world.Step(0.004);
        RequireNear(world.GetBodyTransform(moving).position.y, 0.25, 1e-4, "kinematic body origin");
        // 질량 중심은 물체의 질량이 균형을 이루는 위치로 Body 기준점과 다를 수 있다. 이 중심이 어긋난 회전 물체도 Kinematic 목표 전송을 멈춘 뒤 계속 미끄러지거나 돌지 않아야 한다.
        world.StopKinematic(moving);
        const Transform stopped = world.GetBodyTransform(moving);
        for (int tick = 0; tick < 250; ++tick) world.Step(0.004);
        const Transform afterStop = world.GetBodyTransform(moving);
        RequireNear(glm::length(afterStop.position - stopped.position), 0.0, 1e-5, "stopped kinematic has no linear drift");
        RequireNear(std::abs(glm::dot(afterStop.rotation, stopped.rotation)), 1.0, 1e-5,
            "stopped kinematic has no angular drift");
        ExpectThrows<std::logic_error>([&] { world.StopKinematic(handle); }, "Static body cannot stop as Kinematic");
        world.DestroyBody(moving);

        // Body handle은 소속 World의 고유 표식과 Body ID를 함께 확인한다. World가 파괴된 뒤 같은 ID가 재사용돼도 예전 handle은 새 Body를 가리키지 않는다.
        PhysicsWorld otherWorld;
        const auto other = otherWorld.CreateBody(OffsetPlatform());
        Require(otherWorld.IsBodyValid(other), "own World handle");
        Require(!otherWorld.IsBodyValid(handle), "cross-World handle must fail");
        ExpectThrows<std::invalid_argument>([&] { otherWorld.StopKinematic(handle); }, "cross-World stop must fail");
        ExpectThrows<std::invalid_argument>([&] { otherWorld.GetBodyTransform(handle); }, "cross-World getter must fail");
        world.DestroyBody(handle);
        Require(!world.IsBodyValid(handle), "destroyed handle must fail");
        const auto replacement = world.CreateBody(OffsetPlatform());
        Require(world.IsBodyValid(replacement) && !world.IsBodyValid(handle), "reused ID must not revive old handle");

        // 잘못된 숫자와 회전값은 Jolt 내부로 전달하기 전에 검사해, 호출자가 입력 오류를 예측 가능한 예외로 확인할 수 있게 한다.
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
