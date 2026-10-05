#include "systems/PhysicsSystemModule.h"

#include "PhysicsWorld.h"
#include "components/PhysicsComponents.h"
#include "components/TransformComponents.h"
#include "systems/TransformSystemModule.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <vector>

namespace
{
using grasplink::physics::BoxBodyDescription;
using grasplink::physics::BodyMotionType;
using grasplink::physics::PhysicsBodyHandle;
using grasplink::physics::Transform;
// Fixed Step World 행렬 cache
using WorldTransformCache = std::unordered_map<flecs::entity_t, glm::mat4>;

struct BodySnapshot
{
    flecs::entity entity;
    RigidBody rigidBody;
    BoxCollider collider;
    bool ready = false;
};

glm::mat4 BuildTransformMatrix(const Transform& transform)
{
    return glm::translate(glm::mat4(1.0F), transform.position) *
        glm::mat4_cast(glm::normalize(transform.rotation));
}

glm::mat4 BuildColliderLocalMatrix(const BoxCollider& collider)
{
    return glm::translate(glm::mat4(1.0F), collider.localPositionMeters) *
        glm::mat4_cast(glm::normalize(collider.localRotation));
}

bool IsValidCollider(const BoxCollider& collider)
{
    const glm::vec3& localPosition = collider.localPositionMeters;
    const glm::quat& localRotation = collider.localRotation;
    const float rotationLengthSquared = localRotation.w * localRotation.w +
        localRotation.x * localRotation.x + localRotation.y * localRotation.y +
        localRotation.z * localRotation.z;

    return std::isfinite(localPosition.x) && std::isfinite(localPosition.y) &&
        std::isfinite(localPosition.z) && std::isfinite(localRotation.w) &&
        std::isfinite(localRotation.x) && std::isfinite(localRotation.y) &&
        std::isfinite(localRotation.z) && rotationLengthSquared > 1.0e-8F;
}

// 로봇이 공유하는 부모 행렬
glm::mat4 CalculateWorldMatrix(flecs::entity entity, WorldTransformCache& cache)
{
    if (entity.id() == 0 || !entity.is_alive())
        return glm::mat4(1.0F);

    const flecs::entity_t id = entity.id();
    const auto cached = cache.find(id);
    if (cached != cache.end())
        return cached->second;

    const glm::mat4 local = TransformSystemModule::CalculateLocalMatrix(entity);
    const flecs::entity parent = entity.parent();
    const glm::mat4 world = parent.id() == 0 || !parent.is_alive() || parent == entity
        ? local
        : CalculateWorldMatrix(parent, cache) * local;
    cache.emplace(id, world);
    return world;
}

bool TryGetPhysicsTransform(
    flecs::entity entity,
    const glm::mat4& colliderLocalTransform,
    WorldTransformCache& cache,
    Transform& result)
{
    const glm::mat4 worldMatrix = CalculateWorldMatrix(entity, cache) * colliderLocalTransform;
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

namespace grasplink::viewer::detail
{
struct PhysicsBodyBinding
{
    grasplink::physics::PhysicsBodyHandle handle;
};
}

struct PhysicsSystemModule::Impl
{
    grasplink::physics::PhysicsWorld& physicsWorld;
    flecs::query<const RigidBody, const BoxCollider> bodyQuery;
    flecs::query<const grasplink::viewer::detail::PhysicsBodyBinding> bindingQuery;
    std::vector<BodySnapshot> bodies;
    std::vector<flecs::entity> pendingConfiguration;
    std::vector<flecs::entity> boundEntities;
    flecs::observer rigidBodySetObserver;
    flecs::observer rigidBodyRemoveObserver;
    flecs::observer colliderSetObserver;
    flecs::observer colliderRemoveObserver;
    flecs::observer bodyBindingRemoveObserver;

    Impl(flecs::world& worldValue, grasplink::physics::PhysicsWorld& physicsWorldValue)
        : physicsWorld(physicsWorldValue),
          bodyQuery(worldValue.query<const RigidBody, const BoxCollider>()),
          bindingQuery(worldValue.query<const grasplink::viewer::detail::PhysicsBodyBinding>())
    {
        worldValue.module<PhysicsSystemModule>();

        rigidBodySetObserver = worldValue.observer<RigidBody>()
            .event(flecs::OnSet)
            .each([this](flecs::entity entity, const RigidBody&) { QueueConfiguration(entity); });
        rigidBodyRemoveObserver = worldValue.observer<RigidBody>()
            .event(flecs::OnRemove)
            .each([this](flecs::entity entity, const RigidBody&) { DestroyBody(entity); });
        colliderSetObserver = worldValue.observer<BoxCollider>()
            .event(flecs::OnSet)
            .each([this](flecs::entity entity, const BoxCollider&) { QueueConfiguration(entity); });
        colliderRemoveObserver = worldValue.observer<BoxCollider>()
            .event(flecs::OnRemove)
            .each([this](flecs::entity entity, const BoxCollider&) { DestroyBody(entity); });
        bodyBindingRemoveObserver = worldValue.observer<grasplink::viewer::detail::PhysicsBodyBinding>()
            .event(flecs::OnRemove)
            .each([this](flecs::entity, const grasplink::viewer::detail::PhysicsBodyBinding& binding)
            {
                physicsWorld.DestroyBody(binding.handle);
            });

        WorldTransformCache transformCache;
        bodyQuery.each([this](flecs::entity entity, const RigidBody& rigidBody, const BoxCollider& collider)
        {
            bodies.push_back({entity, rigidBody, collider});
        });
        for (const BodySnapshot& body : bodies)
            ConfigureBody(body.entity, transformCache);
        bodies.clear();
    }

    ~Impl()
    {
        boundEntities.clear();
        bindingQuery.each([this](flecs::entity entity, const grasplink::viewer::detail::PhysicsBodyBinding&)
        {
            boundEntities.push_back(entity);
        });
        for (flecs::entity entity : boundEntities)
            if (entity.is_alive())
                entity.remove<grasplink::viewer::detail::PhysicsBodyBinding>();

        rigidBodySetObserver.destruct();
        rigidBodyRemoveObserver.destruct();
        colliderSetObserver.destruct();
        colliderRemoveObserver.destruct();
        bodyBindingRemoveObserver.destruct();
    }

    // 설정 변경 event 중복 방지
    void QueueConfiguration(flecs::entity entity)
    {
        const bool alreadyQueued = std::any_of(
            pendingConfiguration.begin(), pendingConfiguration.end(),
            [entity](flecs::entity queued) { return queued == entity; });
        if (!alreadyQueued)
            pendingConfiguration.push_back(entity);
    }

    void ConfigureBody(flecs::entity entity, WorldTransformCache& transformCache)
    {
        if (!entity.is_alive())
            return;

        DestroyBody(entity);
        if (!entity.has<RigidBody>() || !entity.has<BoxCollider>())
            return;

        const RigidBody& rigidBody = entity.get<RigidBody>();
        const BoxCollider& collider = entity.get<BoxCollider>();
        if (!IsValidCollider(collider))
        {
            std::cerr << "PhysicsSystemModule: invalid box collider on Entity "
                      << entity.id() << '\n';
            return;
        }

        Transform bodyTransform;
        if (!TryGetPhysicsTransform(entity, BuildColliderLocalMatrix(collider), transformCache, bodyTransform))
        {
            std::cerr << "PhysicsSystemModule: could not calculate Entity World Transform for "
                      << entity.id() << '\n';
            return;
        }

        BoxBodyDescription description;
        description.halfExtentsMeters = collider.halfExtentsMeters;
        description.transform = bodyTransform;
        description.motionType = rigidBody.motionType;

        try
        {
            const PhysicsBodyHandle newHandle = physicsWorld.CreateBox(description);
            entity.set<grasplink::viewer::detail::PhysicsBodyBinding>({newHandle});
        }
        catch (const std::exception& error)
        {
            std::cerr << "PhysicsSystemModule: failed to create Body for Entity "
                      << entity.id() << ": " << error.what() << '\n';
        }
    }

    void DestroyBody(flecs::entity entity)
    {
        if (entity.id() != 0 && entity.is_alive() &&
            entity.has<grasplink::viewer::detail::PhysicsBodyBinding>())
        {
            entity.remove<grasplink::viewer::detail::PhysicsBodyBinding>();
        }
    }

    bool EnsureBody(const BodySnapshot& body, WorldTransformCache& transformCache)
    {
        if (body.entity.has<grasplink::viewer::detail::PhysicsBodyBinding>())
        {
            const auto& binding = body.entity.get<grasplink::viewer::detail::PhysicsBodyBinding>();
            if (physicsWorld.IsBodyValid(binding.handle))
                return true;
            DestroyBody(body.entity);
        }

        ConfigureBody(body.entity, transformCache);
        if (!body.entity.has<grasplink::viewer::detail::PhysicsBodyBinding>())
            return false;
        return physicsWorld.IsBodyValid(
            body.entity.get<grasplink::viewer::detail::PhysicsBodyBinding>().handle);
    }

    void Step(double fixedDeltaSeconds)
    {
        if (!std::isfinite(fixedDeltaSeconds) || fixedDeltaSeconds <= 0.0)
            return;

        // 변경된 Entity 설정을 한 번만 반영
        WorldTransformCache transformCache;
        std::vector<flecs::entity> changedEntities;
        changedEntities.swap(pendingConfiguration);
        for (flecs::entity entity : changedEntities)
            ConfigureBody(entity, transformCache);

        bodies.clear();
        bodyQuery.each([this](flecs::entity entity, const RigidBody& rigidBody, const BoxCollider& collider)
        {
            bodies.push_back({entity, rigidBody, collider});
        });
        for (BodySnapshot& body : bodies)
        {
            if (body.rigidBody.motionType != BodyMotionType::Static)
                body.ready = EnsureBody(body, transformCache);
        }

        // Controller 자세 → Jolt Kinematic Body
        for (const BodySnapshot& body : bodies)
        {
            if (!body.ready || body.rigidBody.motionType != BodyMotionType::Kinematic)
                continue;

            const auto& binding = body.entity.get<grasplink::viewer::detail::PhysicsBodyBinding>();
            Transform targetTransform;
            if (TryGetPhysicsTransform(body.entity, BuildColliderLocalMatrix(body.collider), transformCache, targetTransform))
                physicsWorld.MoveKinematic(binding.handle, targetTransform, fixedDeltaSeconds);
        }

        physicsWorld.Step(fixedDeltaSeconds);

        // Jolt Dynamic pose → Entity Local Transform
        for (const BodySnapshot& body : bodies)
        {
            if (!body.ready || body.rigidBody.motionType != BodyMotionType::Dynamic)
                continue;

            const auto& binding = body.entity.get<grasplink::viewer::detail::PhysicsBodyBinding>();
            ApplyWorldTransform(
                body.entity,
                physicsWorld.GetBodyTransform(binding.handle),
                BuildColliderLocalMatrix(body.collider));
        }
    }

    static void ApplyWorldTransform(
        flecs::entity entity,
        const Transform& bodyTransform,
        const glm::mat4& colliderLocalTransform)
    {
        if (!entity.is_alive())
            return;

        const glm::mat4 parentWorld = TransformSystemModule::CalculateWorldMatrix(entity.parent());
        const glm::mat4 entityWorld = BuildTransformMatrix(bodyTransform) * glm::inverse(colliderLocalTransform);
        const glm::mat4 local = glm::inverse(parentWorld) * entityWorld;

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
