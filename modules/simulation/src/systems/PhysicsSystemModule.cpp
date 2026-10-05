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
    // 바닥 렌더 Mesh 배율은 허용하되 Jolt 형상은 선언된 m 치수를 유지한다.
    return rigidBody.motionType == BodyMotionType::Static &&
        rigidBody.collisionLayer == grasplink::physics::CollisionLayer::Environment;
}

bool HasUnitScale(flecs::entity entity, bool allowEntityScale)
{
    // 현재 Entity와 조상들의 Local Scale pair를 검사한다. Scale이 없는 grouping Entity는 단위로 취급한다.
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
    // Dynamic 부모의 Jolt 이동과 자식 Body 독립 이동을 한 World pose에 합치는 규칙이 없다.
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

    // 갱신된 Scene World 행렬에서 위치·회전만 추출한다. Body 원점은 Entity 원점이며 COM이 아니다.
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
    // ECS에는 비소유 핸들만 둔다. Binding 제거 observer가 PhysicsWorld 소유 Body를 해제한다.
    grasplink::physics::PhysicsBodyHandle handle;
};
}

struct PhysicsSystemModule::Impl
{
    grasplink::physics::PhysicsWorld& physicsWorld;
    flecs::query<const RigidBody, const Colliders> bodyQuery;
    flecs::query<const grasplink::simulation::detail::PhysicsBodyBinding> bindingQuery;
    std::vector<BodySnapshot> bodies;
    // OnSet 안에서는 query를 건드리지 않는다. 다음 Step에서 설정의 최종 상태와 최신 World pose를 읽는다.
    std::vector<flecs::entity> pendingConfiguration;
    std::unordered_set<flecs::entity_t> pendingIds;
    std::vector<flecs::entity> boundEntities;
    // 모든 callback은 this와 physicsWorld를 사용한다. Impl이 없어지기 전에 observer를 철거한다.
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

        // 생성 전에 붙어 있던 설정도 빠뜨리지 않고 연결한다. query 순회와 Binding 추가를 분리한다.
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
        // entity.remove가 동기적으로 OnRemove를 호출하므로 Binding observer가 살아 있을 때 Body를 해제한다.
        // Query를 순회하면서 Binding을 지우면 membership이 바뀌므로 제거 대상 Entity를 먼저 복사한다.
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

    // 같은 fixed 간격 전 두 설정이 연속 변경되어도 ID 집합으로 예약을 합쳐 최종값만 한 번 적용한다.
    void QueueConfiguration(flecs::entity entity)
    {
        if (pendingIds.insert(entity.id()).second)
            pendingConfiguration.push_back(entity);
    }

    bool ConfigureBody(flecs::entity entity)
    {
        if (!entity.is_alive())
            return false;

        // 기존 binding부터 끊어 stale Jolt pose가 새 설정에 남지 않게 한다.
        DestroyBody(entity);
        if (!entity.has<RigidBody>() || !entity.has<Colliders>())
            return false;
        // 설정 pair만으로는 pose가 정의되지 않는다. Entity 자체의 세 Local TRS pair가 모두 필요하다.
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
        // ECS component는 선언 데이터다. Scene World pose와 충돌 필터를 합쳐 Jolt 소유 Body를 만든다.
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
            // OnRemove observer가 handle을 PhysicsWorld에 넘긴다. 여기서 직접 중복 해제하지 않는다.
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

        // Query는 RigidBody+Colliders 조합만 반환한다. Binding 추가/삭제로 query 순회가 무효화되지 않게 snapshot한다.
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

        // Kinematic은 외부 제어 대상: 이번 간격에 움직일 Scene pose를 Jolt 목표로 넘긴다.
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

        // Dynamic은 Jolt 결과가 권위값이다. Body 원점 자세를 Entity Local 위치·회전으로 되돌린다.
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
        // Jolt 위치는 Scene World 기준 Body 원점이다. 부모 World 역행렬로 Local pose를 복원한다.
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

        // Position/Rotation만 물리 결과로 교체한다. Entity의 Local Scale pair와 값은 그대로 둔다.
        entity.set<Position, Local>(Position{translation});
        entity.set<Rotation, Local>(Rotation{glm::normalize(rotation)});
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
