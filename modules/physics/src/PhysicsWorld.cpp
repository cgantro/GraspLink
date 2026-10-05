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

// 전역 Factory/타입 등록은 모든 World가 공유한다. 참조 수와 mutex로 최초 생성 및 최종 해제를 관리한다.
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


// Object Layer에는 상대 범주와 Static/이동 상태를 담는다. Broad Phase 분류는 아래에서 이동 여부만 사용한다.
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


// Broad Phase는 자세한 충돌 계산 전에 후보 쌍을 줄인다. 범주별 허용 여부는 별도 Pair Filter가 정한다.
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


// Robot과 Gripper는 서로 또는 자기 범주와 충돌하지 않고 Environment와 DynamicObject만 상대한다.
// Environment끼리는 충돌하지 않는다. DynamicObject는 모든 범주와 충돌을 허용한다.
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


// Static-Static은 후보에서 제외한다. 이동 Object Layer는 두 Broad Phase 그룹과 모두 검사한다.
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
    // GLM quaternion 생성자는 (w,x,y,z), Jolt는 (x,y,z,w)를 요구한다.
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
    // 필터 interface는 PhysicsSystem이 참조하므로 PhysicsSystem보다 오래 사는 순서로 멤버를 선언한다.
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

        // Jolt Update가 실제로 사용하는 worker pool. 이 설정은 메인 스레드 외 최대 4개 worker를 둔다.
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

        // 좌표: World Y가 위쪽. 중력은 m/s²이며 Body 가속도에 적용된다.
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
    // PhysicsSystem과 참조 자원을 먼저 없앤 뒤 마지막 World인 경우에 전역 등록을 해제한다.
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

    // 유효하지 않은 시간은 상태를 갱신하지 않는다. Fixed Update가 양수 간격을 전달해야 한다.
    // 한 호출을 추가 분할 없이 한 번 계산하며 Jolt 내부 병렬 작업은 jobSystem을 사용한다.
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
    // NaN/무한대 위치와 영 quaternion이 Jolt 공간 계산에 들어가지 않도록 경계에서 거른다.
    // 길이가 1이 아닌 유한 quaternion은 호출 지점에서 정규화하므로 여기서는 0 여부만 검사한다.
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

        // 형상 치수와 정점은 미터이며 GLM XYZ를 Jolt XYZ로 전달한다. 배치는 Body 원점 기준이다.
        compoundSettings.AddShape(ToJoltPosition(part.localTransform.position),
            ToJoltRotation(glm::normalize(part.localTransform.rotation)), shape.GetPtr());
    }

    const auto shapeResult = compoundSettings.Create();
    if (shapeResult.HasError())
        throw std::runtime_error(shapeResult.GetError().c_str());

    const JPH::EMotionType motionType = ToJoltMotionType(description.motionType);
    const JPH::ObjectLayer objectLayer = ToObjectLayer(description.motionType, description.collisionLayer);
    const glm::quat rotation = glm::normalize(description.transform.rotation);
    // 입력은 모델 Body 원점이다. 비대칭 compound의 COM 기준 내부 변환은 Jolt가 처리한다.
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

    // Jolt BodyID의 sequence number가 삭제 후 같은 슬롯에 생성된 새 Body를 구분한다.
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

    // API 계약은 COM 자세가 아니라 모델 Body 원점의 World 자세를 반환한다.
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

    // World Body 원점을 직접 전달한다. COM offset을 더하면 Jolt 내부 보정과 중복된다.
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

    // 목표도 Body 원점 기준이다. Jolt가 COM 이동을 반영하고 목표/간격에서 속도를 계산한다.
    impl_->physicsSystem
        .GetBodyInterface()
        .MoveKinematic(
            bodyID,
            ToJoltPosition(targetTransform.position),
            ToJoltRotation(glm::normalize(targetTransform.rotation)),
            static_cast<float>(fixedDeltaSeconds));
}


void PhysicsWorld::DestroyBody(PhysicsBodyHandle handle)
{
    if (!IsBodyValid(handle))
        return;

    const JPH::BodyID bodyID = ToBodyID(handle);

    JPH::BodyInterface& bodyInterface =
        impl_->physicsSystem.GetBodyInterface();

    // 계산 목록에서 먼저 제거한 뒤 ID와 메모리를 해제해 다음 물리 계산에서 제외한다.
    bodyInterface.RemoveBody(bodyID);

    bodyInterface.DestroyBody(bodyID);
}

} // namespace grasplink::physics
