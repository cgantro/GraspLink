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
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/MutableCompoundShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace grasplink::physics
{

namespace
{

// Jolt Factory와 타입 등록은 모든 PhysicsWorld가 공유하므로 사용 World 수를 세어 관리한다.
// mutex는 여러 thread가 이 수를 동시에 바꾸지 못하게 한다.
// 첫 World를 만들 때 등록하고 마지막 World를 해제할 때 등록을 취소한다.
std::mutex g_JoltRuntimeMutex;
std::size_t g_JoltRuntimeUsers = 0;
std::atomic<std::uint64_t> g_NextWorldToken{1};

void AcquireJoltRuntime()
{
    std::lock_guard<std::mutex> lock(g_JoltRuntimeMutex);

    if (g_JoltRuntimeUsers > 0)
    {
        ++g_JoltRuntimeUsers;
        return;
    }

    JPH::RegisterDefaultAllocator();

    if (JPH::Factory::sInstance != nullptr)
        throw std::runtime_error("Jolt Factory is already initialized.");

    auto factory = std::make_unique<JPH::Factory>();
    JPH::Factory::sInstance = factory.get();
    try
    {
        JPH::RegisterTypes();
    }
    catch (...)
    {
        JPH::UnregisterTypes();
        JPH::Factory::sInstance = nullptr;
        throw;
    }
    factory.release();

    g_JoltRuntimeUsers = 1;
}

void ReleaseJoltRuntime()
{
    std::lock_guard<std::mutex> lock(g_JoltRuntimeMutex);

    if (g_JoltRuntimeUsers == 0)
        return;

    --g_JoltRuntimeUsers;

    if (g_JoltRuntimeUsers > 0)
        return;

    JPH::UnregisterTypes();

    delete JPH::Factory::sInstance;
    JPH::Factory::sInstance = nullptr;
}


// Jolt에 넘기는 내부 그룹 번호는 CollisionLayer 종류와 Static인지 움직이는 Body인지를 함께 담는다. 이 번호는 충돌 필터를 구현하기 위한 값이며 공개 API의 CollisionLayer 자체에 움직임 정보가 추가되는 것은 아니다.
namespace ObjectLayers
{

constexpr JPH::ObjectLayer CategoryCount = 4;
constexpr JPH::ObjectLayer MovingOffset = CategoryCount;
constexpr JPH::ObjectLayer Count = CategoryCount * 2;

JPH::ObjectLayer Make(bool moving, CollisionLayer category)
{
    const JPH::ObjectLayer offset = moving ? MovingOffset : 0;
    return static_cast<JPH::ObjectLayer>(offset + static_cast<JPH::ObjectLayer>(category));
}

bool IsMoving(JPH::ObjectLayer layer)
{
    return layer >= MovingOffset;
}

CollisionLayer Category(JPH::ObjectLayer layer)
{
    return static_cast<CollisionLayer>(layer % CategoryCount);
}

} // namespace ObjectLayers


// Broad Phase는 모든 물체 쌍을 자세히 검사하기 전에 공간상 충돌 가능성이 있는 후보를 빠르게 고르는 단계다. 이 구현은 여기서 물체가 움직이는 그룹인지 정지 그룹인지만 나눈다. Robot과 Environment처럼 어떤 종류끼리 검사할지는 아래 범주 조합 필터가 결정한다.
namespace BroadPhaseLayers
{

constexpr JPH::BroadPhaseLayer NonMoving{0};
constexpr JPH::BroadPhaseLayer Moving{1};
constexpr JPH::uint Count = 2;

} // namespace BroadPhaseLayers


class BroadPhaseLayerInterfaceImpl final
    : public JPH::BroadPhaseLayerInterface
{
public:
    BroadPhaseLayerInterfaceImpl()
    {
        for (JPH::ObjectLayer category = 0; category < ObjectLayers::CategoryCount; ++category)
        {
            objectToBroadPhase_[category] = BroadPhaseLayers::NonMoving;
            objectToBroadPhase_[ObjectLayers::MovingOffset + category] = BroadPhaseLayers::Moving;
        }
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


// 이 함수는 두 CollisionLayer 그룹을 받아 정밀 충돌 검사까지 진행할 물체 쌍인지 결정한다. Robot과 Gripper는 Environment 및 DynamicObject와만 검사하고, 서로 또는 같은 그룹끼리는 검사하지 않는다. Environment끼리는 검사하지 않는다. DynamicObject는 네 그룹 모두와 검사한다.
class ObjectLayerPairFilterImpl final
    : public JPH::ObjectLayerPairFilter
{
public:
    bool ShouldCollide(
        JPH::ObjectLayer object1,
        JPH::ObjectLayer object2) const override
    {
        const CollisionLayer category1 = ObjectLayers::Category(object1);
        const CollisionLayer category2 = ObjectLayers::Category(object2);
        switch (category1)
        {
        case CollisionLayer::Environment:
            return category2 != CollisionLayer::Environment;
        case CollisionLayer::Robot:
        case CollisionLayer::Gripper:
            return category2 == CollisionLayer::Environment ||
                category2 == CollisionLayer::DynamicObject;
        case CollisionLayer::DynamicObject:
            return true;

        default:
            JPH_ASSERT(false);
            return false;
        }
    }
};


// 정지 그룹에 속한 물체는 다른 정지 물체와 검사하지 않는다. 움직이는 Body는 정지 그룹과 움직이는 그룹 모두를 검사해 바닥과 다른 물체 양쪽에 닿을 수 있게 한다.
class ObjectVsBroadPhaseLayerFilterImpl final
    : public JPH::ObjectVsBroadPhaseLayerFilter
{
public:
    bool ShouldCollide(
        JPH::ObjectLayer objectLayer,
        JPH::BroadPhaseLayer broadPhaseLayer) const override
    {
        return ObjectLayers::IsMoving(objectLayer) ||
            broadPhaseLayer == BroadPhaseLayers::Moving;
    }
};



JPH::RVec3 ToJoltPosition(const glm::vec3& value)
{
    return JPH::RVec3(value.x, value.y, value.z);
}

JPH::Quat ToJoltRotation(const glm::quat& value)
{
    // GLM과 Jolt는 같은 회전을 서로 다른 성분 순서로 저장하므로, 이 경계에서 (w,x,y,z)를 (x,y,z,w)로 바꾼다.
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


JPH::ObjectLayer ToObjectLayer(BodyMotionType motionType, CollisionLayer collisionLayer)
{
    return ObjectLayers::Make(motionType != BodyMotionType::Static, collisionLayer);
}


JPH::BodyID ToBodyID(PhysicsBodyHandle handle)
{
    return JPH::BodyID{handle.value};
}


} // namespace


struct PhysicsWorld::Impl
{
    // PhysicsSystem은 아래 필터 객체를 참조만 한다. C++은 멤버를 선언한 역순으로 파괴하므로 필터를 먼저 선언해 PhysicsSystem보다 나중에 해제한다.
    BroadPhaseLayerInterfaceImpl broadPhaseLayerInterface;
    ObjectVsBroadPhaseLayerFilterImpl objectVsBroadPhaseLayerFilter;
    ObjectLayerPairFilterImpl objectLayerPairFilter;

    JPH::PhysicsSystem physicsSystem;

    std::unique_ptr<JPH::TempAllocatorImpl> tempAllocator;

    std::unique_ptr<JPH::JobSystemThreadPool> jobSystem;
    const std::uint64_t worldToken = g_NextWorldToken.fetch_add(1, std::memory_order_relaxed);

    Impl()
    {
        constexpr JPH::uint TempMemorySize = 10U * 1024U * 1024U;

        tempAllocator =
            std::make_unique<JPH::TempAllocatorImpl>(TempMemorySize);

        const unsigned int hardwareThreads =
            std::thread::hardware_concurrency();

        const int workerThreads = hardwareThreads > 1
            ? static_cast<int>(std::min(hardwareThreads - 1, 4U))
            : 1;

        // Jolt Update가 물리 계산을 나눠 실행할 작업자 pool이다. 메인 스레드와 별도로 최대 4개의 worker를 만든다.
        jobSystem = std::make_unique<JPH::JobSystemThreadPool>(
            JPH::cMaxPhysicsJobs,
            JPH::cMaxPhysicsBarriers,
            workerThreads);

        constexpr JPH::uint MaxBodies = 4096;
        constexpr JPH::uint NumBodyMutexes = 0;
        constexpr JPH::uint MaxBodyPairs = 65536;
        constexpr JPH::uint MaxContactConstraints = 10240;

        physicsSystem.Init(
            MaxBodies,
            NumBodyMutexes,
            MaxBodyPairs,
            MaxContactConstraints,
            broadPhaseLayerInterface,
            objectVsBroadPhaseLayerFilter,
            objectLayerPairFilter);

        // 좌표: World의 +Y가 위쪽이다. 중력 벡터의 단위는 m/s²이며 각 Body의 가속도로 적용된다.
        physicsSystem.SetGravity(JPH::Vec3(0.0F, -9.81F, 0.0F));

        JPH::PhysicsSettings settings = physicsSystem.GetPhysicsSettings();
        settings.mPenetrationSlop = 0.002F;
        physicsSystem.SetPhysicsSettings(settings);
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
    // 이 World가 가진 PhysicsSystem과 그 참조 자원을 먼저 해제한다. 살아 있는 World가 마지막 하나일 때만 공유 Jolt 타입 등록을 해제한다.
    impl_.reset();
    ReleaseJoltRuntime();
}


void PhysicsWorld::Step(double fixedDeltaSeconds)
{
    if (!std::isfinite(fixedDeltaSeconds) ||
        fixedDeltaSeconds <= 0.0)
    {
        return;
    }

    // 간격이 0 이하이거나 유한하지 않으면 물리 상태를 바꾸지 않는다. 호출자는 Fixed Update의 양수 시간 간격을 전달해야 한다.
    // 이 호출은 시간을 더 잘게 나누지 않고 Jolt 계산을 한 번 수행한다. Jolt 내부의 병렬 작업은 jobSystem이 맡는다.
    constexpr int CollisionSteps = 1;

    impl_->physicsSystem.Update(
        static_cast<float>(fixedDeltaSeconds),
        CollisionSteps,
        impl_->tempAllocator.get(),
        impl_->jobSystem.get());
}


PhysicsBodyHandle PhysicsWorld::CreateBox(const BoxBodyDescription& description)
{
    CollisionShapeDescription shape;
    shape.halfExtentsMeters = description.halfExtentsMeters;

    BodyDescription body;
    body.shapes.push_back(shape);
    body.transform = description.transform;
    body.motionType = description.motionType;
    body.collisionLayer = description.collisionLayer;
    return CreateBody(body);
}

void ValidateTransform(const Transform& transform)
{
    // NaN이나 무한대가 포함된 위치, 회전을 나타내지 못하는 영 quaternion은 Jolt 계산을 망가뜨릴 수 있어 입력 경계에서 거부한다.
    // 유한한 회전 quaternion은 호출자가 이 지점에 오기 전에 길이 1로 정규화한다. 여기서는 회전 방향이 정의되지 않는 길이 0만 거부한다.
    const float rotationLengthSquared = glm::dot(transform.rotation, transform.rotation);
    if (!std::isfinite(transform.position.x) || !std::isfinite(transform.position.y) ||
        !std::isfinite(transform.position.z) || !std::isfinite(rotationLengthSquared) ||
        rotationLengthSquared <= 1.0e-12F)
        throw std::invalid_argument("Physics Transform requires finite position and nonzero finite rotation");
}

PhysicsBodyHandle PhysicsWorld::CreateBody(const BodyDescription& description)
{
    if (description.shapes.empty())
        throw std::invalid_argument("Physics Body requires at least one collision shape.");
    ValidateTransform(description.transform);
    if (static_cast<unsigned>(description.collisionLayer) > static_cast<unsigned>(CollisionLayer::DynamicObject))
        throw std::invalid_argument("Unsupported CollisionLayer");

    JPH::MutableCompoundShapeSettings compoundSettings;
    for (const CollisionShapeDescription& part : description.shapes)
    {
        ValidateTransform(part.localTransform);
        JPH::RefConst<JPH::Shape> shape;
        switch (part.type)
        {
        case CollisionShapeType::Box:
        {
            if (!std::isfinite(part.halfExtentsMeters.x) || !std::isfinite(part.halfExtentsMeters.y) ||
                !std::isfinite(part.halfExtentsMeters.z) || part.halfExtentsMeters.x <= 0.0F ||
                part.halfExtentsMeters.y <= 0.0F || part.halfExtentsMeters.z <= 0.0F)
                throw std::invalid_argument("Box half extents must be finite and > 0.");
            const float convexRadius = std::min(0.002F,
                std::min(part.halfExtentsMeters.x, std::min(part.halfExtentsMeters.y,
                    part.halfExtentsMeters.z)) * 0.1F);
            shape = new JPH::BoxShape(JPH::Vec3(part.halfExtentsMeters.x,
                part.halfExtentsMeters.y, part.halfExtentsMeters.z), convexRadius);
            break;
        }
        case CollisionShapeType::Cylinder:
            if (!std::isfinite(part.radiusMeters) || !std::isfinite(part.halfHeightMeters) ||
                part.radiusMeters <= 0.0F || part.halfHeightMeters <= 0.0F)
                throw std::invalid_argument("Cylinder radius and half height must be finite and > 0.");
            shape = new JPH::CylinderShape(part.halfHeightMeters, part.radiusMeters,
                std::min(0.002F, std::min(part.halfHeightMeters, part.radiusMeters) * 0.1F));
            break;
        case CollisionShapeType::Sphere:
            if (!std::isfinite(part.radiusMeters) || part.radiusMeters <= 0.0F)
                throw std::invalid_argument("Sphere radius must be finite and > 0.");
            shape = new JPH::SphereShape(part.radiusMeters);
            break;
        case CollisionShapeType::ConvexHull:
        {
            if (part.pointsMeters.size() < 4)
                throw std::invalid_argument("Convex hull requires at least four points.");
            JPH::Array<JPH::Vec3> points;
            points.reserve(part.pointsMeters.size());
            for (const glm::vec3& point : part.pointsMeters)
            {
                if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                    throw std::invalid_argument("Convex hull points must be finite.");
                points.emplace_back(point.x, point.y, point.z);
            }
            JPH::ConvexHullShapeSettings hullSettings(points, 0.0F);
            const auto hullResult = hullSettings.Create();
            if (hullResult.HasError())
                throw std::runtime_error(hullResult.GetError().c_str());
            shape = hullResult.Get();
            break;
        }
        default:
            throw std::invalid_argument("Unsupported CollisionShapeType");
        }

        // 형상 치수와 정점은 미터 단위다. GLM의 X/Y/Z 값을 같은 축의 Jolt 좌표로 전달하고, 각 형상은 모델 Body 원점에 대해 배치한다.
        compoundSettings.AddShape(ToJoltPosition(part.localTransform.position),
            ToJoltRotation(glm::normalize(part.localTransform.rotation)), shape.GetPtr());
    }

    const auto shapeResult = compoundSettings.Create();
    if (shapeResult.HasError())
        throw std::runtime_error(shapeResult.GetError().c_str());

    const JPH::EMotionType motionType = ToJoltMotionType(description.motionType);
    const JPH::ObjectLayer objectLayer = ToObjectLayer(description.motionType, description.collisionLayer);
    const glm::quat rotation = glm::normalize(description.transform.rotation);
    // 입력 자세는 모델에서 정한 Body 원점이다. 여러 형상의 배치가 비대칭이라 무게중심(COM)이 달라져도 Jolt가 내부 좌표 변환을 맡는다.
    JPH::BodyCreationSettings settings(shapeResult.Get(),
        ToJoltPosition(description.transform.position), ToJoltRotation(rotation),
        motionType, objectLayer);
    JPH::BodyInterface& bodyInterface = impl_->physicsSystem.GetBodyInterface();
    const JPH::EActivation activation = description.motionType == BodyMotionType::Static
        ? JPH::EActivation::DontActivate
        : JPH::EActivation::Activate;
    const JPH::BodyID bodyID = bodyInterface.CreateAndAddBody(settings, activation);
    if (bodyID.IsInvalid())
        throw std::runtime_error("Failed to create physics body.");

    return {bodyID.GetIndexAndSequenceNumber(), impl_->worldToken};
}


bool PhysicsWorld::IsBodyValid(PhysicsBodyHandle handle) const
{
    if (!handle.IsValid() || handle.worldToken != impl_->worldToken)
        return false;

    const JPH::BodyID bodyID = ToBodyID(handle);

    // Jolt는 삭제된 Body의 슬롯을 재사용할 수 있다. BodyID의 sequence number까지 비교해 같은 슬롯에 새로 만들어진 Body를 오래된 핸들과 구분한다.
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

    // 이 API는 Jolt가 계산에 쓰는 무게중심(COM) 자세 대신, 모델과 Entity가 공유하는 Body 원점의 World 자세를 반환한다.
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
    ValidateTransform(transform);

    // 모델 Body 원점의 World 자세를 그대로 전달한다. 무게중심(COM) 차이를 여기서 다시 더하면 Jolt의 내부 보정과 겹쳐 위치가 어긋난다.
    impl_->physicsSystem
        .GetBodyInterface()
        .SetPositionAndRotation(
            ToBodyID(handle),
            ToJoltPosition(transform.position),
            ToJoltRotation(glm::normalize(transform.rotation)),
            JPH::EActivation::Activate);
}


void PhysicsWorld::MoveKinematic(
    PhysicsBodyHandle handle,
    const Transform& targetTransform,
    double fixedDeltaSeconds)
{
    if (!IsBodyValid(handle))
        throw std::invalid_argument("Invalid PhysicsBodyHandle.");
    ValidateTransform(targetTransform);

    if (!std::isfinite(fixedDeltaSeconds) ||
        fixedDeltaSeconds <= 0.0)
    {
        throw std::invalid_argument(
            "fixedDeltaSeconds must be finite and > 0.");
    }

    const JPH::BodyID bodyID = ToBodyID(handle);

    const JPH::EMotionType motionType =
        impl_->physicsSystem
            .GetBodyInterface()
            .GetMotionType(bodyID);

    if (motionType != JPH::EMotionType::Kinematic)
    {
        throw std::logic_error(
            "MoveKinematic requires a kinematic body.");
    }

    // 목표 자세 역시 모델 Body 원점 기준이다. Jolt가 무게중심(COM) 이동을 내부에서 처리하고 목표와 시간 간격으로 필요한 속도를 계산한다.
    impl_->physicsSystem
        .GetBodyInterface()
        .MoveKinematic(
            bodyID,
            ToJoltPosition(targetTransform.position),
            ToJoltRotation(glm::normalize(targetTransform.rotation)),
            static_cast<float>(fixedDeltaSeconds));
}


void PhysicsWorld::StopKinematic(PhysicsBodyHandle handle)
{
    if (!IsBodyValid(handle))
        throw std::invalid_argument("Invalid PhysicsBodyHandle.");
    auto& bodyInterface = impl_->physicsSystem.GetBodyInterface();
    const JPH::BodyID bodyID = ToBodyID(handle);
    if (bodyInterface.GetMotionType(bodyID) != JPH::EMotionType::Kinematic)
        throw std::logic_error("StopKinematic requires a kinematic body.");
    bodyInterface.SetLinearAndAngularVelocity(bodyID, JPH::Vec3::sZero(), JPH::Vec3::sZero());
}

void PhysicsWorld::DestroyBody(PhysicsBodyHandle handle)
{
    if (!IsBodyValid(handle))
        return;

    const JPH::BodyID bodyID = ToBodyID(handle);

    JPH::BodyInterface& bodyInterface =
        impl_->physicsSystem.GetBodyInterface();

    // 먼저 Jolt 계산 목록에서 Body를 제거해 다음 Step에 참여하지 않게 한다. 이후 ID와 Body 메모리를 해제한다.
    bodyInterface.RemoveBody(bodyID);

    bodyInterface.DestroyBody(bodyID);
}

} // namespace grasplink::physics
