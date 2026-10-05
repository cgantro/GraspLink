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

// Jolt 전역 자원은 첫 World에서 만들고 마지막 World가 사라질 때 해제한다.
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


// Object Layer: 충돌 상대 분류 4개에 Static/이동 여부를 조합한다.
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


// Broad Phase: 자세한 충돌 계산 전 후보를 고르는 단계. Static과 이동 Body를 나눈다.
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


// Robot/Gripper 자기 충돌과 Environment끼리 충돌은 제외한다.
// Robot/Gripper는 Environment와 DynamicObject에만 닿는다.
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


// Static끼리는 움직이지 않으므로 충돌 후보 검사에서 제외한다.
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
    // Quaternion 생성자 순서: GLM (w,x,y,z) -> Jolt (x,y,z,w).
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
    // PhysicsSystem이 이 객체들을 참조하므로 PhysicsSystem보다 오래 살아 있어야 한다.
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

        // 좌표: World Y가 위쪽. 중력 단위는 m/s².
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
    // PhysicsSystem을 먼저 제거한 후 Jolt 전역 자원을 정리한다.
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

    // FixedControlLoop의 짧은 고정 간격을 추가 분할 없이 한 번 계산한다.
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

        compoundSettings.AddShape(ToJoltPosition(part.localTransform.position),
            ToJoltRotation(glm::normalize(part.localTransform.rotation)), shape.GetPtr());
    }

    const auto shapeResult = compoundSettings.Create();
    if (shapeResult.HasError())
        throw std::runtime_error(shapeResult.GetError().c_str());

    const JPH::EMotionType motionType = ToJoltMotionType(description.motionType);
    const JPH::ObjectLayer objectLayer = ToObjectLayer(description.motionType, description.collisionLayer);
    const glm::quat rotation = glm::normalize(description.transform.rotation);
    // 입력은 모델의 Body 원점. Compound 형상 COM으로 옮기는 계산은 Jolt 내부 담당.
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

    // ID의 sequence number로 삭제 후 같은 슬롯에 생성된 새 Body를 구분한다.
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

    // GetPositionAndRotation은 COM이 아닌 Body 원점의 World 자세를 돌려준다.
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

    // World 좌표의 Body 원점을 전달한다. COM offset을 별도로 더하지 않는다.
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

    // 목표도 Body 원점 기준. COM 이동과 속도 계산은 Jolt가 처리한다.
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

    // 계산 목록에서 먼저 빼야 해당 Body의 ID와 메모리를 해제할 수 있다.
    bodyInterface.RemoveBody(bodyID);

    bodyInterface.DestroyBody(bodyID);
}

} // namespace grasplink::physics
