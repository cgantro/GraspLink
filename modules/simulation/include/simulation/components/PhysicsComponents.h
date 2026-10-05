#pragma once

#include "PhysicsTypes.h"

#include <glm/gtc/quaternion.hpp>

#include <vector>

/**
 * @brief Entity가 사용할 물리 Body의 이동 방식과 충돌 범주를 선언한다.
 * @details ECS 설정은 Body를 직접 소유하지 않는다. PhysicsSystemModule이 Colliders와 함께 읽어
 * PhysicsWorld에 Jolt Body를 만들고, 설정 변경 시 기존 Body를 해제한 뒤 다시 연결한다.
 * Dynamic은 Jolt 결과가 Entity로 돌아오고, Kinematic은 Entity World 자세가 목표가 되며,
 * Static은 시뮬레이션 중 고정된다. CollisionLayer는 이동 방식과 별개의 필터 범주다.
 */
struct RigidBody
{
    // Dynamic: Jolt 결과를 반영. Kinematic: Entity World 자세를 목표로 전달. Static: 고정 환경.
    grasplink::physics::BodyMotionType motionType =
        grasplink::physics::BodyMotionType::Dynamic;

    // 실제 접촉 여부는 PhysicsWorld의 범주 쌍 필터가 결정한다.
    grasplink::physics::CollisionLayer collisionLayer =
        grasplink::physics::CollisionLayer::DynamicObject;
};

/**
 * @brief 한 Entity의 Body 원점에 배치할 충돌 형상 목록.
 * @details 치수와 localTransform 위치는 m 단위이며 형상은 Entity의 시각 Mesh와 별도 데이터다.
 * Entity 계층의 scale은 형상 치수에 곱해지지 않는다. 여러 형상은 하나의 Body를 공유하며
 * Jolt 내부 무게중심(COM)이 Body 원점과 달라도 ECS와 PhysicsWorld는 모델 Body 원점을 사용한다.
 */
struct Colliders
{
    std::vector<grasplink::physics::CollisionShapeDescription> shapes;
};

namespace physics_colliders
{
/**
 * @brief Body 원점 기준 Box 형상을 설명한다.
 * @param halfExtentsMeters 축별 반쪽 길이 [m].
 * @param localPositionMeters Body 원점에서 Box 중심까지의 위치 [m].
 * @param localRotation Body 원점 기준 Box 회전. GLM quaternion 순서는 (w,x,y,z).
 */
inline grasplink::physics::CollisionShapeDescription Box(
    const glm::vec3& halfExtentsMeters,
    const glm::vec3& localPositionMeters = glm::vec3{0.0F},
    const glm::quat& localRotation = glm::quat{1.0F, 0.0F, 0.0F, 0.0F})
{
    grasplink::physics::CollisionShapeDescription shape;
    shape.halfExtentsMeters = halfExtentsMeters;
    shape.localTransform.position = localPositionMeters;
    shape.localTransform.rotation = localRotation;
    return shape;
}

/**
 * @brief Body 원점 기준 Y축 Cylinder 형상을 설명한다.
 * @param radiusMeters 반지름 [m].
 * @param halfHeightMeters Y축 반쪽 높이 [m].
 * @param localPositionMeters Body 원점에서 Cylinder 중심까지의 위치 [m].
 * @param localRotation Body 원점 기준 형상 회전. GLM quaternion 순서는 (w,x,y,z).
 */
inline grasplink::physics::CollisionShapeDescription Cylinder(
    float radiusMeters,
    float halfHeightMeters,
    const glm::vec3& localPositionMeters = glm::vec3{0.0F},
    const glm::quat& localRotation = glm::quat{1.0F, 0.0F, 0.0F, 0.0F})
{
    grasplink::physics::CollisionShapeDescription shape;
    shape.type = grasplink::physics::CollisionShapeType::Cylinder;
    shape.radiusMeters = radiusMeters;
    shape.halfHeightMeters = halfHeightMeters;
    shape.localTransform.position = localPositionMeters;
    shape.localTransform.rotation = localRotation;
    return shape;
}

/**
 * @brief Body 원점 기준 Sphere 형상을 설명한다.
 * @param radiusMeters 반지름 [m].
 * @param localPositionMeters Body 원점에서 Sphere 중심까지의 위치 [m].
 */
inline grasplink::physics::CollisionShapeDescription Sphere(
    float radiusMeters,
    const glm::vec3& localPositionMeters = glm::vec3{0.0F})
{
    grasplink::physics::CollisionShapeDescription shape;
    shape.type = grasplink::physics::CollisionShapeType::Sphere;
    shape.radiusMeters = radiusMeters;
    shape.localTransform.position = localPositionMeters;
    return shape;
}
}
