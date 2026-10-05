#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdint>
#include <vector>

namespace grasplink::physics
{

// Body ID와 World 소유자를 함께 보관한다. Body 자체는 PhysicsWorld가 소유한다.
struct PhysicsBodyHandle
{
    static constexpr std::uint32_t InvalidValue = 0xFFFFFFFFU;

    std::uint32_t value = InvalidValue;
    // 같은 Body ID라도 다른 World의 handle은 사용할 수 없다.
    std::uint64_t worldToken = 0;

    [[nodiscard]]
    bool IsValid() const noexcept
    {
        return value != InvalidValue && worldToken != 0;
    }
};


// Body 자세는 World 기준, 형상 자세는 Body 원점 기준. 위치 단위는 m.
// Body 원점은 모델의 기준점이며 형상들의 무게중심(COM)과 다를 수 있다.
struct Transform
{
    glm::vec3 position{0.0F};

    glm::quat rotation{
        1.0F,
        0.0F,
        0.0F,
        0.0F
    };
};


// Static은 고정, Kinematic은 외부 목표 자세, Dynamic은 Jolt 계산으로 움직인다.
enum class BodyMotionType : std::uint8_t
{
    Static,
    Kinematic,
    Dynamic
};

// 충돌 상대 분류. Motion Type과 조합해 움직이는 물체 여부도 구분한다.
enum class CollisionLayer : std::uint8_t
{
    Environment,
    Robot,
    Gripper,
    DynamicObject
};

enum class CollisionShapeType : std::uint8_t
{
    Box,
    Cylinder,
    Sphere,
    ConvexHull
};

// 한 형상의 크기와 Body 원점 기준 배치. 여러 형상을 한 Body에 넣을 수 있다.
struct CollisionShapeDescription
{
    CollisionShapeType type = CollisionShapeType::Box;
    // Box 반쪽 크기 [m]. {0.5, 0.5, 0.5}이면 전체 크기는 1 m.
    glm::vec3 halfExtentsMeters{0.5F};
    // 반지름과 Cylinder 반쪽 높이 [m]. Cylinder 축은 형상의 Local Y.
    float radiusMeters = 0.1F;
    float halfHeightMeters = 0.1F;
    // 형상 원점과 회전: Body 원점 기준. 크기는 scale 대신 위 치수로 지정한다.
    Transform localTransform;
    // ConvexHull 정점 [m]: 형상 Local 기준. localTransform으로 Body에 배치한다.
    std::vector<glm::vec3> pointsMeters;
};

struct BodyDescription
{
    // 모든 형상이 같은 Body 자세와 움직임을 공유한다.
    std::vector<CollisionShapeDescription> shapes;
    Transform transform;
    BodyMotionType motionType = BodyMotionType::Dynamic;
    CollisionLayer collisionLayer = CollisionLayer::DynamicObject;
};


struct BoxBodyDescription
{
    // Box 반쪽 크기 [m].
    glm::vec3 halfExtentsMeters{
        0.5F,
        0.5F,
        0.5F
    };

    Transform transform;

    BodyMotionType motionType =
        BodyMotionType::Dynamic;

    CollisionLayer collisionLayer =
        CollisionLayer::DynamicObject;
};

} // namespace grasplink::physics
