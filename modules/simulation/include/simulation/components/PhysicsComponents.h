#pragma once

#include "PhysicsTypes.h"

#include <glm/gtc/quaternion.hpp>

#include <vector>

// Entity의 물리 설정. 실제 Jolt Body 수명은 PhysicsSystemModule이 관리한다.
struct RigidBody
{
    // Dynamic: Jolt 결과 반영. Kinematic: Controller 목표 전달. Static: 생성 위치 유지.
    grasplink::physics::BodyMotionType motionType =
        grasplink::physics::BodyMotionType::Dynamic;

    // Robot/Gripper 자기 충돌은 제외하고 환경·DynamicObject와 충돌한다.
    grasplink::physics::CollisionLayer collisionLayer =
        grasplink::physics::CollisionLayer::DynamicObject;
};

// 한 Entity의 Body 원점 기준 형상 모음. 치수는 m이며 Entity scale을 적용하지 않는다.
struct Colliders
{
    std::vector<grasplink::physics::CollisionShapeDescription> shapes;
};

namespace physics_colliders
{
// 입력: 반쪽 길이·반지름은 m. 형상 위치·회전은 Entity의 Body 원점 기준.
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
