#include "PhysicsWorld.h"

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>

#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>

#include <cmath>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace grasplink::physics
{

namespace
{

/*
 * Jolt의 Factory와 Type 등록은 전역 자원이다.
 *
 * PhysicsWorld가 여러 개 생성되어도 최초 1회만 초기화하고,
 * 마지막 PhysicsWorld가 제거될 때 정리한다.
 */
std::mutex g_JoltRuntimeMutex;
std::size_t g_JoltRuntimeUsers = 0;

void AcquireJoltRuntime()
{
    std::lock_guard<std::mutex> lock(g_JoltRuntimeMutex);

    // 이미 다른 PhysicsWorld가 Jolt를 사용 중이면 다시 초기화하지 않는다.
    if (g_JoltRuntimeUsers > 0)
    {
        ++g_JoltRuntimeUsers;
        return;
    }

    // 1. Jolt 기본 메모리 allocator 등록.
    JPH::RegisterDefaultAllocator();

    // 2. Jolt 내부 객체 생성에 사용하는 Factory 생성.
    if (JPH::Factory::sInstance != nullptr)
        throw std::runtime_error("Jolt Factory is already initialized.");

    JPH::Factory::sInstance = new JPH::Factory();

    // 3. Box, Constraint 등 Jolt 기본 타입 등록.
    JPH::RegisterTypes();

    g_JoltRuntimeUsers = 1;
}

void ReleaseJoltRuntime()
{
    std::lock_guard<std::mutex> lock(g_JoltRuntimeMutex);

    if (g_JoltRuntimeUsers == 0)
        return;

    --g_JoltRuntimeUsers;

    // 아직 다른 PhysicsWorld가 사용 중이면 Jolt를 종료하지 않는다.
    if (g_JoltRuntimeUsers > 0)
        return;

    JPH::UnregisterTypes();

    delete JPH::Factory::sInstance;
    JPH::Factory::sInstance = nullptr;
}


/*
 * Collision Layer
 *
 * NonMoving:
 * - Floor
 * - Wall
 * - Static Body
 *
 * Moving:
 * - Dynamic Body
 * - Kinematic Body
 */
namespace ObjectLayers
{

constexpr JPH::ObjectLayer NonMoving = 0;
constexpr JPH::ObjectLayer Moving = 1;
constexpr JPH::ObjectLayer Count = 2;

} // namespace ObjectLayers


/*
 * Broad Phase는 실제 충돌 검사를 하기 전에
 * 충돌 가능성이 있는 Body 후보를 빠르게 찾는 단계다.
 */
namespace BroadPhaseLayers
{

constexpr JPH::BroadPhaseLayer NonMoving{0};
constexpr JPH::BroadPhaseLayer Moving{1};
constexpr JPH::uint Count = 2;

} // namespace BroadPhaseLayers


/*
 * Object Layer를 Broad Phase Layer로 연결한다.
 *
 * NonMoving -> NonMoving
 * Moving    -> Moving
 */
class BroadPhaseLayerInterfaceImpl final
    : public JPH::BroadPhaseLayerInterface
{
public:
    BroadPhaseLayerInterfaceImpl()
    {
        objectToBroadPhase_[ObjectLayers::NonMoving] = BroadPhaseLayers::NonMoving;
        objectToBroadPhase_[ObjectLayers::Moving] = BroadPhaseLayers::Moving;
    }

    JPH::uint GetNumBroadPhaseLayers() const override
    {
        return BroadPhaseLayers::Count;
    }

    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override
    {
        JPH_ASSERT(layer < ObjectLayers::Count);
        return objectToBroadPhase_[layer];
    }

#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)

    const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer layer) const override
    {
        if (layer == BroadPhaseLayers::NonMoving)
            return "NON_MOVING";

        if (layer == BroadPhaseLayers::Moving)
            return "MOVING";

        return "INVALID";
    }

#endif

private:
    JPH::BroadPhaseLayer objectToBroadPhase_[ObjectLayers::Count];
};


/*
 * 실제 Body 종류끼리 충돌 가능한지 정한다.
 *
 * Static vs Static -> X
 * Static vs Moving -> O
 * Moving vs Static -> O
 * Moving vs Moving -> O
 */
class ObjectLayerPairFilterImpl final
    : public JPH::ObjectLayerPairFilter
{
public:
    bool ShouldCollide(
        JPH::ObjectLayer object1,
        JPH::ObjectLayer object2) const override
    {
        switch (object1)
        {
        case ObjectLayers::NonMoving:
            return object2 == ObjectLayers::Moving;

        case ObjectLayers::Moving:
            return true;

        default:
            JPH_ASSERT(false);
            return false;
        }
    }
};


/*
 * Broad Phase 단계에서 검사할 Layer를 정한다.
 */
class ObjectVsBroadPhaseLayerFilterImpl final
    : public JPH::ObjectVsBroadPhaseLayerFilter
{
public:
    bool ShouldCollide(
        JPH::ObjectLayer objectLayer,
        JPH::BroadPhaseLayer broadPhaseLayer) const override
    {
        switch (objectLayer)
        {
        case ObjectLayers::NonMoving:
            return broadPhaseLayer == BroadPhaseLayers::Moving;

        case ObjectLayers::Moving:
            return true;

        default:
            JPH_ASSERT(false);
            return false;
        }
    }
};


/*
 * GLM <-> Jolt 변환
 */

JPH::RVec3 ToJoltPosition(const glm::vec3& value)
{
    return JPH::RVec3(value.x, value.y, value.z);
}

JPH::Quat ToJoltRotation(const glm::quat& value)
{
    // GLM 생성자 순서:  (w, x, y, z)
    // Jolt 생성자 순서: (x, y, z, w)
    return JPH::Quat(value.x, value.y, value.z, value.w);
}

glm::vec3 ToGlmPosition(const JPH::RVec3& value)
{
    return {
        static_cast<float>(value.GetX()),
        static_cast<float>(value.GetY()),
        static_cast<float>(value.GetZ())
    };
}

glm::quat ToGlmRotation(const JPH::Quat& value)
{
    return {
        value.GetW(),
        value.GetX(),
        value.GetY(),
        value.GetZ()
    };
}


/*
 * GraspLink BodyMotionType -> Jolt MotionType
 */
JPH::EMotionType ToJoltMotionType(BodyMotionType motionType)
{
    switch (motionType)
    {
    case BodyMotionType::Static:
        return JPH::EMotionType::Static;

    case BodyMotionType::Kinematic:
        return JPH::EMotionType::Kinematic;

    case BodyMotionType::Dynamic:
        return JPH::EMotionType::Dynamic;
    }

    throw std::invalid_argument("Unsupported BodyMotionType.");
}


/*
 * Static Body와 움직이는 Body는 다른 Collision Layer를 사용한다.
 */
JPH::ObjectLayer ToObjectLayer(BodyMotionType motionType)
{
    if (motionType == BodyMotionType::Static)
        return ObjectLayers::NonMoving;

    return ObjectLayers::Moving;
}


/*
 * PhysicsBodyHandle -> Jolt BodyID
 */
JPH::BodyID ToBodyID(PhysicsBodyHandle handle)
{
    return JPH::BodyID{handle.value};
}


/*
 * Box의 half extent가 정상적인지 검사한다.
 */
void ValidateBoxDescription(const BoxBodyDescription& description)
{
    const glm::vec3& size = description.halfExtentsMeters;

    if (!std::isfinite(size.x) ||
        !std::isfinite(size.y) ||
        !std::isfinite(size.z) ||
        size.x <= 0.0F ||
        size.y <= 0.0F ||
        size.z <= 0.0F)
    {
        throw std::invalid_argument(
            "Box half extents must be finite and > 0.");
    }
}

} // namespace


/*
 * PhysicsWorld 내부 구현.
 *
 * Jolt 타입을 PhysicsWorld.h 밖으로 노출하지 않기 위해
 * PImpl 구조를 사용한다.
 */
struct PhysicsWorld::Impl
{
    // PhysicsSystem이 이 객체들을 참조하므로 PhysicsSystem보다 오래 살아 있어야 한다.
    BroadPhaseLayerInterfaceImpl broadPhaseLayerInterface;
    ObjectVsBroadPhaseLayerFilterImpl objectVsBroadPhaseLayerFilter;
    ObjectLayerPairFilterImpl objectLayerPairFilter;

    // 실제 Jolt Physics World.
    JPH::PhysicsSystem physicsSystem;

    // Physics 계산 중 사용하는 임시 메모리.
    std::unique_ptr<JPH::TempAllocatorImpl> tempAllocator;

    // Physics 작업을 병렬로 실행하는 Job System.
    std::unique_ptr<JPH::JobSystemThreadPool> jobSystem;

    Impl()
    {
        // 1. Physics 계산용 임시 메모리.
        constexpr JPH::uint TempMemorySize = 10U * 1024U * 1024U;

        tempAllocator =
            std::make_unique<JPH::TempAllocatorImpl>(TempMemorySize);

        // 2. CPU thread 수를 기준으로 worker 수를 정한다.
        const unsigned int hardwareThreads =
            std::thread::hardware_concurrency();

        const int workerThreads =
            hardwareThreads > 1
                ? static_cast<int>(hardwareThreads - 1)
                : 1;

        jobSystem = std::make_unique<JPH::JobSystemThreadPool>(
            JPH::cMaxPhysicsJobs,
            JPH::cMaxPhysicsBarriers,
            workerThreads);

        // 3. 초기 Physics World 최대 용량.
        constexpr JPH::uint MaxBodies = 4096;
        constexpr JPH::uint NumBodyMutexes = 0;
        constexpr JPH::uint MaxBodyPairs = 65536;
        constexpr JPH::uint MaxContactConstraints = 10240;

        // 4. PhysicsSystem 초기화.
        physicsSystem.Init(
            MaxBodies,
            NumBodyMutexes,
            MaxBodyPairs,
            MaxContactConstraints,
            broadPhaseLayerInterface,
            objectVsBroadPhaseLayerFilter,
            objectLayerPairFilter);

        // 5. Y-Up 기준 중력 설정.
        physicsSystem.SetGravity(JPH::Vec3(0.0F, -9.81F, 0.0F));
    }
};


PhysicsWorld::PhysicsWorld()
{
    AcquireJoltRuntime();

    try
    {
        impl_ = std::make_unique<Impl>();
    }
    catch (...)
    {
        ReleaseJoltRuntime();
        throw;
    }
}

PhysicsWorld::~PhysicsWorld()
{
    // PhysicsSystem을 먼저 제거한 후 Jolt 전역 자원을 정리한다.
    impl_.reset();
    ReleaseJoltRuntime();
}


void PhysicsWorld::Step(double fixedDeltaSeconds)
{
    // 0, 음수, NaN, Inf는 정상적인 timestep이 아니다.
    if (!std::isfinite(fixedDeltaSeconds) ||
        fixedDeltaSeconds <= 0.0)
    {
        return;
    }

    // FixedControlLoop에서 이미 작은 고정 dt로 호출하므로
    // 현재는 collision step을 1로 둔다.
    constexpr int CollisionSteps = 1;

    impl_->physicsSystem.Update(
        static_cast<float>(fixedDeltaSeconds),
        CollisionSteps,
        impl_->tempAllocator.get(),
        impl_->jobSystem.get());
}


PhysicsBodyHandle PhysicsWorld::CreateBox(
    const BoxBodyDescription& description)
{
    ValidateBoxDescription(description);

    // 1. BoxShape는 전체 크기가 아니라 half extent를 사용한다.
    JPH::RefConst<JPH::Shape> shape = new JPH::BoxShape(
        JPH::Vec3(
            description.halfExtentsMeters.x,
            description.halfExtentsMeters.y,
            description.halfExtentsMeters.z));

    // 2. GraspLink 타입을 Jolt 타입으로 변환한다.
    const JPH::EMotionType motionType =
        ToJoltMotionType(description.motionType);

    const JPH::ObjectLayer objectLayer =
        ToObjectLayer(description.motionType);

    // 3. Body 생성 설정.
    JPH::BodyCreationSettings settings(
        shape,
        ToJoltPosition(description.transform.position),
        ToJoltRotation(description.transform.rotation),
        motionType,
        objectLayer);

    JPH::BodyInterface& bodyInterface =
        impl_->physicsSystem.GetBodyInterface();

    // Static은 깨울 필요가 없고 Dynamic/Kinematic은 바로 활성화한다.
    const JPH::EActivation activation =
        description.motionType == BodyMotionType::Static
            ? JPH::EActivation::DontActivate
            : JPH::EActivation::Activate;

    const JPH::BodyID bodyID =
        bodyInterface.CreateAndAddBody(settings, activation);

    if (bodyID.IsInvalid())
        throw std::runtime_error("Failed to create physics body.");

    // Jolt BodyID 자체 대신 uint32 기반 Handle을 반환한다.
    return PhysicsBodyHandle{
        bodyID.GetIndexAndSequenceNumber()
    };
}


bool PhysicsWorld::IsBodyValid(PhysicsBodyHandle handle) const
{
    if (!handle.IsValid())
        return false;

    const JPH::BodyID bodyID = ToBodyID(handle);

    // sequence number도 포함하므로 삭제 후 재사용된 Body와 구분할 수 있다.
    return impl_->physicsSystem
        .GetBodyInterface()
        .IsAdded(bodyID);
}


Transform PhysicsWorld::GetBodyTransform(
    PhysicsBodyHandle handle) const
{
    if (!IsBodyValid(handle))
        throw std::invalid_argument("Invalid PhysicsBodyHandle.");

    const JPH::BodyID bodyID = ToBodyID(handle);

    JPH::RVec3 position;
    JPH::Quat rotation;

    impl_->physicsSystem
        .GetBodyInterface()
        .GetPositionAndRotation(bodyID, position, rotation);

    Transform result;
    result.position = ToGlmPosition(position);
    result.rotation = ToGlmRotation(rotation);

    return result;
}


void PhysicsWorld::SetBodyTransform(
    PhysicsBodyHandle handle,
    const Transform& transform)
{
    if (!IsBodyValid(handle))
        throw std::invalid_argument("Invalid PhysicsBodyHandle.");

    // 이동 과정을 계산하지 않고 Body를 즉시 해당 Transform으로 옮긴다.
    impl_->physicsSystem
        .GetBodyInterface()
        .SetPositionAndRotation(
            ToBodyID(handle),
            ToJoltPosition(transform.position),
            ToJoltRotation(transform.rotation),
            JPH::EActivation::Activate);
}


void PhysicsWorld::MoveKinematic(
    PhysicsBodyHandle handle,
    const Transform& targetTransform,
    double fixedDeltaSeconds)
{
    if (!IsBodyValid(handle))
        throw std::invalid_argument("Invalid PhysicsBodyHandle.");

    if (!std::isfinite(fixedDeltaSeconds) ||
        fixedDeltaSeconds <= 0.0)
    {
        throw std::invalid_argument(
            "fixedDeltaSeconds must be finite and > 0.");
    }

    const JPH::BodyID bodyID = ToBodyID(handle);

    // Kinematic Body가 아닌 경우 잘못된 API 사용이므로 차단한다.
    const JPH::EMotionType motionType =
        impl_->physicsSystem
            .GetBodyInterface()
            .GetMotionType(bodyID);

    if (motionType != JPH::EMotionType::Kinematic)
    {
        throw std::logic_error(
            "MoveKinematic requires a kinematic body.");
    }

    // fixedDeltaSeconds 동안 목표 Transform으로 이동할 속도를 Jolt가 계산한다.
    impl_->physicsSystem
        .GetBodyInterface()
        .MoveKinematic(
            bodyID,
            ToJoltPosition(targetTransform.position),
            ToJoltRotation(targetTransform.rotation),
            static_cast<float>(fixedDeltaSeconds));
}


void PhysicsWorld::DestroyBody(PhysicsBodyHandle handle)
{
    if (!IsBodyValid(handle))
        return;

    const JPH::BodyID bodyID = ToBodyID(handle);

    JPH::BodyInterface& bodyInterface =
        impl_->physicsSystem.GetBodyInterface();

    // 1. Simulation에서 Body 제거.
    bodyInterface.RemoveBody(bodyID);

    // 2. Body ID와 메모리 해제.
    bodyInterface.DestroyBody(bodyID);
}

} // namespace grasplink::physics