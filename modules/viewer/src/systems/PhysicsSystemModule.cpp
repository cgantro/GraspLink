#include "systems/PhysicsSystemModule.h"

#include "PhysicsWorld.h"
#include "components/PhysicsComponents.h"
#include "components/TransformComponents.h"
#include "systems/TransformSystemModule.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/quaternion.hpp>

#include <cmath>
#include <iostream>
#include <vector>

namespace
{
using grasplink::physics::BoxBodyDescription;
using grasplink::physics::BodyMotionType;
using grasplink::physics::PhysicsBodyHandle;
using grasplink::physics::Transform;

// Body 재생성 때 Component 추가 → Flecs 내부 구성 변경 가능
// 순회 중 목록 변경 방지를 위해 처리 값 복사
struct BodySnapshot
{
    flecs::entity entity;
    RigidBody rigidBody;
    BoxCollider collider;
};

// PhysicsWorld 위치·회전: Scene 기준
glm::mat4 BuildTransformMatrix(const Transform& transform)
{
    return glm::translate(glm::mat4(1.0F), transform.position) *
        glm::mat4_cast(glm::normalize(transform.rotation));
}

// Box 중심 offset: Entity Local 기준
glm::mat4 BuildColliderLocalMatrix(const BoxCollider& collider)
{
    return glm::translate(glm::mat4(1.0F), collider.localPositionMeters) *
        glm::mat4_cast(glm::normalize(collider.localRotation));
}

// Box 크기·회전 유효성 검사. 잘못된 값은 Jolt Shape 생성 실패
bool IsValidCollider(const BoxCollider& collider)
{
    const glm::vec3& halfExtents = collider.halfExtentsMeters;
    const glm::vec3& localPosition = collider.localPositionMeters;
    const glm::quat& localRotation = collider.localRotation;
    const float rotationLengthSquared = localRotation.w * localRotation.w +
        localRotation.x * localRotation.x + localRotation.y * localRotation.y +
        localRotation.z * localRotation.z;

    return std::isfinite(halfExtents.x) && std::isfinite(halfExtents.y) &&
        std::isfinite(halfExtents.z) && halfExtents.x > 0.0F &&
        halfExtents.y > 0.0F && halfExtents.z > 0.0F &&
        std::isfinite(localPosition.x) && std::isfinite(localPosition.y) &&
        std::isfinite(localPosition.z) && std::isfinite(localRotation.w) &&
        std::isfinite(localRotation.x) && std::isfinite(localRotation.y) &&
        std::isfinite(localRotation.z) && rotationLengthSquared > 1.0e-8F;
}

// Entity·부모 변환 + Collider offset → Jolt에 보낼 Scene 기준 위치·회전
bool TryGetPhysicsTransform(
    flecs::entity entity,
    const glm::mat4& colliderLocalTransform,
    Transform& result)
{
    const glm::mat4 worldMatrix =
        TransformSystemModule::CalculateWorldMatrix(entity) * colliderLocalTransform;
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
// 내부 연결 Component. Entity 생성 코드에는 노출하지 않음
struct PhysicsBodyBinding
{
    grasplink::physics::PhysicsBodyHandle handle;
};
}

struct PhysicsSystemModule::Impl
{
    // 참조만 보관. PhysicsSystemModule 파괴 후 World·PhysicsWorld 파괴
    flecs::world& world;
    grasplink::physics::PhysicsWorld& physicsWorld;
    flecs::query<const RigidBody, const BoxCollider> bodyQuery;
    flecs::query<const grasplink::viewer::detail::PhysicsBodyBinding> bindingQuery;
    std::vector<BodySnapshot> bodies;
    std::vector<flecs::entity> boundEntities;
    flecs::observer rigidBodySetObserver;
    flecs::observer rigidBodyRemoveObserver;
    flecs::observer colliderSetObserver;
    flecs::observer colliderRemoveObserver;
    flecs::observer bodyBindingRemoveObserver;

    Impl(flecs::world& worldValue, grasplink::physics::PhysicsWorld& physicsWorldValue)
        : world(worldValue),
          physicsWorld(physicsWorldValue),
          bodyQuery(world.query<const RigidBody, const BoxCollider>()),
          bindingQuery(world.query<const grasplink::viewer::detail::PhysicsBodyBinding>())
    {
        world.module<PhysicsSystemModule>();

        // 설정 추가·변경 → Jolt Body 재생성
        // 설정 제거 → Body 삭제
        rigidBodySetObserver = world.observer<RigidBody>()
            .event(flecs::OnSet)
            .each([this](flecs::entity entity, const RigidBody&)
            {
                ConfigureBody(entity);
            });

        rigidBodyRemoveObserver = world.observer<RigidBody>()
            .event(flecs::OnRemove)
            .each([this](flecs::entity entity, const RigidBody&)
            {
                DestroyBody(entity);
            });

        colliderSetObserver = world.observer<BoxCollider>()
            .event(flecs::OnSet)
            .each([this](flecs::entity entity, const BoxCollider&)
            {
                ConfigureBody(entity);
            });

        colliderRemoveObserver = world.observer<BoxCollider>()
            .event(flecs::OnRemove)
            .each([this](flecs::entity entity, const BoxCollider&)
            {
                DestroyBody(entity);
            });

        bodyBindingRemoveObserver = world.observer<grasplink::viewer::detail::PhysicsBodyBinding>()
            .event(flecs::OnRemove)
            .each([this](flecs::entity, const grasplink::viewer::detail::PhysicsBodyBinding& binding)
            {
                // 연결 Component 제거 → Jolt Body 삭제. Entity 삭제 때도 동일 경로
                physicsWorld.DestroyBody(binding.handle);
            });

        // 모듈 생성 전 물리 설정이 붙은 Entity도 Body 생성 대상
        bodyQuery.each([this](flecs::entity entity, const RigidBody& rigidBody, const BoxCollider& collider)
        {
            bodies.push_back({entity, rigidBody, collider});
        });
        for (const BodySnapshot& body : bodies)
            ConfigureBody(body.entity);
        bodies.clear();
    }

    ~Impl()
    {
        // PhysicsWorld 생존 중 Entity 연결 제거 → Body 삭제
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

    // RigidBody + BoxCollider + 유효한 값 → Body 생성. 잘못된 값은 로그 후 건너뜀
    void ConfigureBody(flecs::entity entity)
    {
        if (!entity.is_alive())
            return;

        // 이전 설정의 Body 삭제 후 새 설정 확인
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
        if (!TryGetPhysicsTransform(entity, BuildColliderLocalMatrix(collider), bodyTransform))
        {
            std::cerr << "PhysicsSystemModule: could not calculate Entity World Transform for "
                      << entity.id() << '\n';
            return;
        }

        BoxBodyDescription description;
        // 크기 단위: 미터. Entity Scale과 별도로 지정
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

    // 연결 정보 제거 → 감시자가 Jolt Body도 삭제
    void DestroyBody(flecs::entity entity)
    {
        if (entity.id() == 0 || !entity.is_alive() ||
            !entity.has<grasplink::viewer::detail::PhysicsBodyBinding>())
        {
            return;
        }

        entity.remove<grasplink::viewer::detail::PhysicsBodyBinding>();
    }

    // 고정 Step: 로봇 자세 전달 → Jolt 계산 → Dynamic Entity 갱신
    void Step(double fixedDeltaSeconds)
    {
        if (!std::isfinite(fixedDeltaSeconds) || fixedDeltaSeconds <= 0.0)
            return;

        // Body 복구 중 Entity 구성이 바뀔 수 있어 처리 목록 먼저 복사
        bodies.clear();
        bodyQuery.each([this](flecs::entity entity, const RigidBody& rigidBody, const BoxCollider& collider)
        {
            bodies.push_back({entity, rigidBody, collider});
        });

        // Controller 관절 위치: 부모 기준 → 부모 변환을 반영해 Jolt에 전달
        for (const BodySnapshot& body : bodies)
        {
            if (!EnsureBody(body))
                continue;

            const auto& binding =
                body.entity.get<grasplink::viewer::detail::PhysicsBodyBinding>();
            if (body.rigidBody.motionType != BodyMotionType::Kinematic)
                continue;

            Transform targetTransform;
            if (!TryGetPhysicsTransform(body.entity, BuildColliderLocalMatrix(body.collider), targetTransform))
                continue;

            physicsWorld.MoveKinematic(binding.handle, targetTransform, fixedDeltaSeconds);
        }

        physicsWorld.Step(fixedDeltaSeconds);

        // Jolt 결과: Scene 기준 → 부모 변환을 되돌려 Entity Local 위치·회전에 저장
        for (const BodySnapshot& body : bodies)
        {
            if (body.rigidBody.motionType != BodyMotionType::Dynamic || !EnsureBody(body))
                continue;

            const auto& binding =
                body.entity.get<grasplink::viewer::detail::PhysicsBodyBinding>();

            ApplyWorldTransform(
                body.entity,
                physicsWorld.GetBodyTransform(binding.handle),
                BuildColliderLocalMatrix(body.collider));
        }
    }

    // Jolt Body 유효성 확인. 사라졌으면 현재 Component 설정으로 재생성
    bool EnsureBody(const BodySnapshot& body)
    {
        if (!body.entity.has<grasplink::viewer::detail::PhysicsBodyBinding>())
            return false;

        const auto& binding =
            body.entity.get<grasplink::viewer::detail::PhysicsBodyBinding>();
        if (physicsWorld.IsBodyValid(binding.handle))
            return true;

        DestroyBody(body.entity);
        ConfigureBody(body.entity);
        return body.entity.has<grasplink::viewer::detail::PhysicsBodyBinding>();
    }

    // Jolt Box 중심: Scene 기준 위치·회전
    // Collider offset과 부모 변환을 되돌려 Entity Local 값으로 변환
    // Entity Scale은 Physics 계산에서 제외
    static void ApplyWorldTransform(
        flecs::entity entity,
        const Transform& bodyTransform,
        const glm::mat4& colliderLocalTransform)
    {
        if (!entity.is_alive())
            return;

        const glm::mat4 parentWorld =
            TransformSystemModule::CalculateWorldMatrix(entity.parent());
        const glm::mat4 entityWorld =
            BuildTransformMatrix(bodyTransform) * glm::inverse(colliderLocalTransform);
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
