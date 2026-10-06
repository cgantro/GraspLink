#pragma once

#include "PhysicsTypes.h"

#include <glm/gtc/quaternion.hpp>

#include <vector>

/**
 * @brief Entity에 연결할 Body의 움직임 방식과 충돌 범주를 ECS에 기록한다.
 * @details Physics Body는 Jolt가 위치, 속도, 충돌을 계산하는 물체다.
 * 이 component는 Body 설정만 저장하며 실제 Body는 PhysicsWorld가 소유한다.
 * Entity의 Local 위치와 회전은 바로 위 부모를 기준으로 하고, World 값은 부모 계층을 합친 장면 전체 기준이다.
 * PhysicsSystemModule은 이 설정과 Colliders를 읽어 Body를 만들고, 설정이 바뀌면 이전 Body를 해제한 뒤 다시 만든다.
 * Dynamic은 Jolt가 계산한 자세를 Entity에 돌려준다.
 * Kinematic은 Entity의 World 자세를 목표로 움직인다.
 * Static은 시뮬레이션 중 움직이지 않는다.
 * CollisionLayer는 움직임 방식과 별개로 어떤 종류의 물체끼리 충돌을 검사할지 정한다.
 */
struct RigidBody
{
    // Dynamic은 Jolt가 계산한 자세를 Entity에 돌려준다. Kinematic은 Entity World 자세를 목표로 따라간다. Static은 시뮬레이션 중 고정된 환경 물체다.
    grasplink::physics::BodyMotionType motionType =
        grasplink::physics::BodyMotionType::Dynamic;

    // 이 범주는 충돌 가능 대상을 고른다. 예를 들어 Robot과 Environment의 접촉을 허용할지는 PhysicsWorld의 범주 조합 필터가 결정한다.
    grasplink::physics::CollisionLayer collisionLayer =
        grasplink::physics::CollisionLayer::DynamicObject;
};

/**
 * @brief 한 Entity의 물리 Body에 붙일 충돌 모양과 Body 기준 배치를 모아 둔다.
 * @details Collider는 접촉 여부를 계산하는 모양이며 화면에 보이는 Mesh와 별도로 지정한다.
 * 각 모양의 크기와 localTransform 위치는 [m]이고 Entity나 부모의 scale은 모양 크기에 곱해지지 않는다.
 * 여러 모양은 하나의 Body를 이루므로 같은 위치와 움직임 방식을 공유한다.
 * 모양이 Entity 원점에서 치우치면 Jolt 계산용 무게중심(COM)이 원점과 달라질 수 있다.
 * ECS와 PhysicsWorld는 계속 모델 기준 Body 원점을 자세를 주고받는 기준으로 사용한다.
 */
struct Colliders
{
    std::vector<grasplink::physics::CollisionShapeDescription> shapes;
};

namespace physics_colliders
{
/**
 * @brief Body 원점에서의 중심 위치와 회전, 축별 반쪽 길이로 Box 충돌 형상을 지정한다.
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
 * @brief Body 원점에서의 중심 위치와 회전, 반지름과 높이로 원통 충돌 형상을 지정한다. 기본 축은 형상 Local Y다.
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
 * @brief Body 원점에서 중심 위치와 반지름으로 구 충돌 형상을 지정한다.
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
