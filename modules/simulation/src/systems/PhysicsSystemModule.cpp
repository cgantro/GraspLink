#include "simulation/systems/PhysicsSystemModule.h"

#include "PhysicsWorld.h"
#include "simulation/components/PhysicsComponents.h"
#include "components/TransformComponents.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_set>
#include <vector>

namespace
{
using grasplink::physics::BodyDescription;
using grasplink::physics::BodyMotionType;
using grasplink::physics::PhysicsBodyHandle;
using grasplink::physics::Transform;

struct BodySnapshot
{
    flecs::entity entity;
    bool ready = false;
};

glm::mat4 BuildTransformMatrix(const Transform& transform)
{
    return glm::translate(glm::mat4(1.0F), transform.position) *
        glm::mat4_cast(glm::normalize(transform.rotation));
}

bool AllowsVisualScale(const RigidBody& rigidBody)
{
    // Static Environment 자신의 scale만 시각용으로 허용한다. Collider는 지정한 m 치수 유지.
    return rigidBody.motionType == BodyMotionType::Static &&
        rigidBody.collisionLayer == grasplink::physics::CollisionLayer::Environment;
}

bool HasUnitScale(flecs::entity entity, bool allowEntityScale)
{
    // 부모 scale도 collider에 전파되지 않으므로 물리 계층에서는 단위 scale을 요구한다.
    bool isEntity = true;
    while (entity.id() != 0 && entity.is_alive())
    {
        if ((!isEntity || !allowEntityScale) && entity.has<Scale, Local>())
        {
            const glm::vec3 scale = entity.get<Scale, Local>();
            if (glm::any(glm::greaterThan(glm::abs(scale - glm::vec3{1.0F}), glm::vec3{1.0e-4F})))
                return false;
        }

        const flecs::entity parent = entity.parent();
        if (parent.id() == 0 || !parent.is_alive() || parent == entity)
            break;
        entity = parent;
        isEntity = false;
    }
    return true;
}

bool HasSupportedPhysicsParents(flecs::entity entity)
{
    // Dynamic 조상의 갱신 순서와 자식의 독립 물리 운동은 아직 정의하지 않는다.
    for (flecs::entity parent = entity.parent(); parent.id() != 0 && parent.is_alive(); parent = parent.parent())
        if (parent.has<RigidBody>() && parent.get<RigidBody>().motionType == BodyMotionType::Dynamic)
            return false;
    return true;
}

bool TryGetPhysicsTransform(
    flecs::entity entity,
    Transform& result)
{
    if (!entity.has<TransformMatrix, World>())
        return false;

    // 입력은 Scene World 변환. 출력 위치는 Entity와 같은 Body 원점이며 COM이 아니다.
    const glm::mat4 worldMatrix = entity.get<TransformMatrix, World>();
    glm::vec3 scale{1.0F};
    glm::quat rotation{1.0F, 0.0F, 0.0F, 0.0F};
    glm::vec3 translation{0.0F};
    glm::vec3 skew{0.0F};
    glm::vec4 perspective{0.0F};

    if (!glm::decompose(worldMatrix, scale, rotation, translation, skew, perspective))
        return false;

    result.position = translation;
    result.rotation = glm::normalize(rotation);
    return true;
}
}

namespace grasplink::simulation
{
namespace detail
{
struct PhysicsBodyBinding
{
    // Body 소유자는 PhysicsWorld. 이 연결을 제거하면 observer가 Body도 제거한다.
    grasplink::physics::PhysicsBodyHandle handle;
};
}

struct PhysicsSystemModule::Impl
{
    grasplink::physics::PhysicsWorld& physicsWorld;
    flecs::query<const RigidBody, const Colliders> bodyQuery;
    flecs::query<const grasplink::simulation::detail::PhysicsBodyBinding> bindingQuery;
    std::vector<BodySnapshot> bodies;
    // OnSet에서는 설정만 예약한다. 다음 Step에서 최신 World 자세로 Body를 다시 만든다.
    std::vector<flecs::entity> pendingConfiguration;
    std::unordered_set<flecs::entity_t> pendingIds;
    std::vector<flecs::entity> boundEntities;
    // Callback이 this를 참조하므로 Impl이 사라지기 전에 모두 해제해야 한다.
    flecs::observer rigidBodySetObserver;
    flecs::observer rigidBodyRemoveObserver;
    flecs::observer colliderSetObserver;
    flecs::observer colliderRemoveObserver;
    flecs::observer bodyBindingRemoveObserver;

    Impl(flecs::world& worldValue, grasplink::physics::PhysicsWorld& physicsWorldValue)
        : physicsWorld(physicsWorldValue),
          bodyQuery(worldValue.query<const RigidBody, const Colliders>()),
          bindingQuery(worldValue.query<const grasplink::simulation::detail::PhysicsBodyBinding>())
    {
        worldValue.module<PhysicsSystemModule>();

        rigidBodySetObserver = worldValue.observer<RigidBody>()
            .event(flecs::OnSet)
            .each([this](flecs::entity entity, const RigidBody&) { QueueConfiguration(entity); });
        rigidBodyRemoveObserver = worldValue.observer<RigidBody>()
            .event(flecs::OnRemove)
            .each([this](flecs::entity entity, const RigidBody&) { DestroyBody(entity); });
        colliderSetObserver = worldValue.observer<Colliders>()
            .event(flecs::OnSet)
            .each([this](flecs::entity entity, const Colliders&) { QueueConfiguration(entity); });
        colliderRemoveObserver = worldValue.observer<Colliders>()
            .event(flecs::OnRemove)
            .each([this](flecs::entity entity, const Colliders&) { DestroyBody(entity); });
        bodyBindingRemoveObserver = worldValue.observer<detail::PhysicsBodyBinding>()
            .event(flecs::OnRemove)
            .each([this](flecs::entity, const grasplink::simulation::detail::PhysicsBodyBinding& binding)
            {
                physicsWorld.DestroyBody(binding.handle);
            });

        bodyQuery.each([this](flecs::entity entity, const RigidBody&, const Colliders&)
        {
            bodies.push_back({entity});
        });
        for (const BodySnapshot& body : bodies)
            ConfigureBody(body.entity);
        bodies.clear();
    }

    ~Impl()
    {
        // BodyBinding 제거 callback이 유효한 동안 Body를 먼저 정리한다.
        boundEntities.clear();
        bindingQuery.each([this](flecs::entity entity, const detail::PhysicsBodyBinding&)
        {
            boundEntities.push_back(entity);
        });
        for (flecs::entity entity : boundEntities)
            if (entity.is_alive())
                entity.remove<grasplink::simulation::detail::PhysicsBodyBinding>();

        rigidBodySetObserver.destruct();
        rigidBodyRemoveObserver.destruct();
        colliderSetObserver.destruct();
        colliderRemoveObserver.destruct();
        bodyBindingRemoveObserver.destruct();
    }

    // 같은 Entity의 RigidBody/Colliders가 연달아 바뀌어도 한 번만 재구성한다.
    void QueueConfiguration(flecs::entity entity)
    {
        if (pendingIds.insert(entity.id()).second)
            pendingConfiguration.push_back(entity);
    }

    bool ConfigureBody(flecs::entity entity)
    {
        if (!entity.is_alive())
            return false;

        DestroyBody(entity);
        if (!entity.has<RigidBody>() || !entity.has<Colliders>())
            return false;
        if (!entity.has<Position, Local>() || !entity.has<Rotation, Local>() || !entity.has<Scale, Local>())
            return false;

        const RigidBody& rigidBody = entity.get<RigidBody>();
        const Colliders& colliders = entity.get<Colliders>();
        if (colliders.shapes.empty())
            return false;
        if (!HasUnitScale(entity, AllowsVisualScale(rigidBody)) || !HasSupportedPhysicsParents(entity))
        {
            std::cerr << "PhysicsSystemModule: Physics hierarchy requires unit scale and no Dynamic ancestor: "
                      << entity.id() << '\n';
            return false;
        }
        Transform bodyTransform;
        if (!TryGetPhysicsTransform(entity, bodyTransform))
        {
            std::cerr << "PhysicsSystemModule: could not calculate Entity World Transform for "
                      << entity.id() << '\n';
            return false;
        }

        BodyDescription description;
        description.shapes = colliders.shapes;
        description.transform = bodyTransform;
        description.motionType = rigidBody.motionType;
        description.collisionLayer = rigidBody.collisionLayer;

        try
        {
            const PhysicsBodyHandle newHandle = physicsWorld.CreateBody(description);
            entity.set<grasplink::simulation::detail::PhysicsBodyBinding>({newHandle});
            return true;
        }
        catch (const std::exception& error)
        {
            std::cerr << "PhysicsSystemModule: failed to create Body for Entity "
                      << entity.id() << ": " << error.what() << '\n';
            return false;
        }
    }

    void DestroyBody(flecs::entity entity)
    {
        if (entity.id() != 0 && entity.is_alive() &&
            entity.has<grasplink::simulation::detail::PhysicsBodyBinding>())
        {
            entity.remove<grasplink::simulation::detail::PhysicsBodyBinding>();
        }
    }

    bool EnsureBody(flecs::entity entity, bool configurationChanged)
    {
        if (!configurationChanged && entity.has<grasplink::simulation::detail::PhysicsBodyBinding>())
        {
            const auto& binding = entity.get<grasplink::simulation::detail::PhysicsBodyBinding>();
            if (physicsWorld.IsBodyValid(binding.handle))
                return true;
            DestroyBody(entity);
        }

        return ConfigureBody(entity);
    }

    void Step(double fixedDeltaSeconds)
    {
        if (!std::isfinite(fixedDeltaSeconds) || fixedDeltaSeconds <= 0.0)
            return;

        // 설정 변경 -> Body 재구성 -> Kinematic 목표 -> Jolt 계산 -> Dynamic 결과.
        std::vector<flecs::entity> changedEntities;
        changedEntities.swap(pendingConfiguration);
        for (flecs::entity entity : changedEntities)
        {
            if (!entity.is_alive() || !entity.has<RigidBody>() || !entity.has<Colliders>())
                DestroyBody(entity);
        }

        // Query를 순회하는 동안 component를 바꾸지 않도록 Entity 목록을 먼저 모은다.
        bodies.clear();
        bodyQuery.each([this](flecs::entity entity, const RigidBody&, const Colliders&)
        {
            bodies.push_back({entity});
        });
        for (BodySnapshot& body : bodies)
        {
            const RigidBody& rigidBody = body.entity.get<RigidBody>();
            if (!body.entity.has<Position, Local>() || !body.entity.has<Rotation, Local>() ||
                !body.entity.has<Scale, Local>())
            {
                DestroyBody(body.entity);
                continue;
            }
            if (!HasUnitScale(body.entity, AllowsVisualScale(rigidBody)) || !HasSupportedPhysicsParents(body.entity))
            {
                DestroyBody(body.entity);
                continue;
            }

            body.ready = EnsureBody(body.entity, pendingIds.erase(body.entity.id()) != 0);
        }
        pendingIds.clear();

        // Controller가 갱신한 Entity World 자세를 이번 간격의 Kinematic 목표로 보낸다.
        for (const BodySnapshot& body : bodies)
        {
            if (!body.ready || body.entity.get<RigidBody>().motionType != BodyMotionType::Kinematic)
                continue;

            const auto& binding = body.entity.get<grasplink::simulation::detail::PhysicsBodyBinding>();
            Transform targetTransform;
            if (TryGetPhysicsTransform(body.entity, targetTransform))
                physicsWorld.MoveKinematic(binding.handle, targetTransform, fixedDeltaSeconds);
        }

        physicsWorld.Step(fixedDeltaSeconds);

        // Jolt가 움직인 Dynamic Body의 원점 자세를 Entity Local 값으로 되돌린다.
        for (const BodySnapshot& body : bodies)
        {
            if (!body.ready || body.entity.get<RigidBody>().motionType != BodyMotionType::Dynamic)
                continue;

            const auto& binding = body.entity.get<grasplink::simulation::detail::PhysicsBodyBinding>();
            ApplyWorldTransform(
                body.entity,
                physicsWorld.GetBodyTransform(binding.handle));
        }
    }

    static void ApplyWorldTransform(
        flecs::entity entity,
        const Transform& bodyTransform)
    {
        if (!entity.is_alive())
            return;

        const flecs::entity parent = entity.parent();
        // 좌표: Jolt의 물체 원점 World pose에서 부모 변환을 되돌려 Local로 저장한다.
        const glm::mat4 parentWorld = parent.id() != 0 && parent.is_alive() && parent.has<TransformMatrix, World>()
            ? parent.get<TransformMatrix, World>()
            : glm::mat4(1.0F);
        const glm::mat4 local = glm::inverse(parentWorld) * BuildTransformMatrix(bodyTransform);

        glm::vec3 scale{1.0F};
        glm::quat rotation{1.0F, 0.0F, 0.0F, 0.0F};
        glm::vec3 translation{0.0F};
        glm::vec3 skew{0.0F};
        glm::vec4 perspective{0.0F};
        if (!glm::decompose(local, scale, rotation, translation, skew, perspective))
            return;

        entity.set<Position, Local>(Position{translation});
        entity.set<Rotation, Local>(Rotation{glm::eulerAngles(glm::normalize(rotation))});
    }
};

PhysicsSystemModule::PhysicsSystemModule(
    flecs::world& world,
    grasplink::physics::PhysicsWorld& physicsWorld)
    : m_Impl(std::make_unique<Impl>(world, physicsWorld))
{
}

PhysicsSystemModule::~PhysicsSystemModule() = default;

void PhysicsSystemModule::Step(double fixedDeltaSeconds)
{
    m_Impl->Step(fixedDeltaSeconds);
}

}
