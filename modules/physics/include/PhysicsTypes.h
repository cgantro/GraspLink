#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdint>
#include <vector>

namespace grasplink::physics
{

/**
 * @brief PhysicsWorld 안의 Body를 가리키는 값 핸들.
 * @details
 * Body 메모리와 Jolt ID는 PhysicsWorld가 소유한다. 핸들에는 World별 token도 들어 있어
 * 다른 World의 우연히 같은 ID에 접근하지 않으며, Jolt ID의 sequence 정보로 재사용된 슬롯도 구분한다.
 * Body를 파괴한 뒤에는 핸들을 다시 사용하지 않는다.
 */
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


/**
 * @brief 위치와 회전을 함께 표현하는 자세.
 * @details
 * Body 자세의 위치는 World 기준이고, CollisionShapeDescription의 localTransform은 Body 원점 기준이다.
 * 위치 단위는 m이며, Body 원점은 모델의 기준점이다. 형상 배치가 비대칭이면 무게중심(COM)은
 * 이 원점과 다를 수 있다. GLM quaternion은 생성자에서 (w,x,y,z) 순서를 쓴다.
 */
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


/**
 * @brief Body의 이동을 결정하는 방식.
 * @details Static은 고정 물체로 취급하고, Kinematic은 외부 목표 자세를 따라간다.
 * Dynamic은 충돌과 중력 계산 결과로 Jolt가 움직이며, 외부 시스템은 결과 자세를 읽어 반영한다.
 */
enum class BodyMotionType : std::uint8_t
{
    Static,
    Kinematic,
    Dynamic
};

/**
 * @brief 충돌 필터가 사용하는 상대 분류.
 * @details
 * 범주는 Motion Type과 별개다. 구현은 범주에 Static/이동 상태를 더해 Jolt Object Layer를 만들고,
 * 범주 쌍 필터로 실제 충돌 허용 여부를 정한다. Broad Phase Layer는 별도로 움직임 여부만 분류해
 * 자세한 충돌 후보를 줄인다.
 */
enum class CollisionLayer : std::uint8_t
{
    Environment,
    Robot,
    Gripper,
    DynamicObject
};

/** @brief Body에 넣을 수 있는 충돌 형상 종류. */
enum class CollisionShapeType : std::uint8_t
{
    Box,
    Cylinder,
    Sphere,
    ConvexHull
};

/**
 * @brief Body에 포함할 한 충돌 형상의 치수와 배치.
 * @details 충돌 형상은 렌더링 Mesh와 별도 데이터다. 치수와 정점은 m 단위이며 렌더링 형상의 모양이나
 * scale에서 자동으로 만들어지지 않는다. 여러 형상은 하나의 Body를 공유하고 localTransform으로 배치한다.
 */
struct CollisionShapeDescription
{
    CollisionShapeType type = CollisionShapeType::Box;
    // Box 반쪽 크기 [m]. {0.5, 0.5, 0.5}이면 전체 크기는 1 m.
    glm::vec3 halfExtentsMeters{0.5F};
    // Sphere/Cylinder 반지름과 Cylinder 반쪽 높이 [m]. Cylinder 축은 형상의 Local Y.
    float radiusMeters = 0.1F;
    float halfHeightMeters = 0.1F;
    // 형상 원점과 회전: Body 원점 기준. GLM quaternion 순서는 (w,x,y,z); 크기는 scale 대신 위 치수로 지정한다.
    Transform localTransform;
    // ConvexHull 정점 [m]: 형상 Local 기준. 4개 이상의 유한한 점이 필요하며 localTransform으로 Body에 배치한다.
    std::vector<glm::vec3> pointsMeters;
};

/**
 * @brief 복수 충돌 형상을 하나의 물리 Body로 생성하기 위한 설명.
 * @details 모든 형상은 한 자세와 Motion Type을 공유한다. transform.position은 형상 COM이 아니라 모델의
 * Body 원점이다. Jolt가 compound의 COM 기준 내부 표현을 계산하며 공개 API는 Body 원점 자세를 사용한다.
 */
struct BodyDescription
{
    // 모든 형상이 같은 Body 자세와 움직임을 공유한다.
    std::vector<CollisionShapeDescription> shapes;
    Transform transform;
    BodyMotionType motionType = BodyMotionType::Dynamic;
    CollisionLayer collisionLayer = CollisionLayer::DynamicObject;
};


/** @brief 단일 Box Body 생성에 쓰는 간단한 설명. halfExtentsMeters는 각 축의 반쪽 길이다. */
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
