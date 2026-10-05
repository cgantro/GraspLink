#include "Entity.h"
#include "PhysicsWorld.h"
#include "TestSupport.h"
#include "assets/GltfLoader.h"
#include "robotics/backends/simulation/SimGripperController.h"
#include "robotics/kinematics/GripperKinematics.h"
#include "robotics/kinematics/RobotKinematics.h"
#include "robotics/models/hanwha/Hcr12a.h"
#include "robotics/models/robotiq/TwoF85.h"
#include "scene/SceneManager.h"
#include "simulation/components/PhysicsComponents.h"
#include "simulation/robotics/GripperColliders.h"
#include "simulation/systems/PhysicsSystemModule.h"
#include "systems/TransformSystemModule.h"
#include "viewer/robotics/GripperTransformAdapter.h"
#include "viewer/robotics/RobotTransformAdapter.h"

#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace
{
using namespace grasplink::robotics;
using grasplink::physics::BodyMotionType;
using grasplink::physics::CollisionLayer;

void RequireVector(const glm::vec3& actual, const glm::vec3& expected, const std::string& label)
{
    // GLB/Entity는 float, 기구학은 double을 사용한다. 위치의 20 μm 오차로 변환 정밀도 차이를 허용한다.
    for (int axis = 0; axis < 3; ++axis)
        RequireNear(actual[axis], expected[axis], 2.0e-5, label);
}

void RequireMatrix(const glm::mat4& actual, const glm::mat4& expected, const std::string& label)
{
    for (int column = 0; column < 4; ++column)
        for (int row = 0; row < 4; ++row)
            RequireNear(actual[column][row], expected[column][row], 2.0e-5, label);
}

glm::vec3 InGripper(const Entity& entity, const Entity& gripper)
{
    return glm::vec3(glm::inverse(gripper.GetWorldMatrix()) * entity.GetWorldMatrix()[3]);
}

/**
 * @brief 실제 GLB 전체 계층에서 Controller→관절→World→Jolt 흐름을 실행한다.
 * @details GPU 업로드 없이 원본 노드의 TRS와 부모 관계를 보존한다. Scene이 Entity를 소유하고,
 * 어댑터와 물리 연결은 Scene·World보다 먼저 정리된다. 팔 FK와 그리퍼 분기 회전은 같은 4 ms tick에 적용한다.
 */
struct Fixture
{
    flecs::world world;
    grasplink::physics::PhysicsWorld physics;
    SceneManager scenes{world};
    Scene* scene = nullptr;
    ModelResource model;
    std::vector<Entity> nodes;
    Entity robotRoot;
    Entity gripper;
    backends::simulation::SimGripperController controller{models::robotiq::kTwoF85};
    kinematics::GripperKinematics gripperMath{models::robotiq::kTwoF85};
    kinematics::RobotKinematics armMath{models::hanwha::kHcr12a};
    RobotState armState;
    std::unique_ptr<grasplink::viewer::robotics::RobotTransformAdapter> armAdapter;
    std::unique_ptr<grasplink::viewer::robotics::GripperTransformAdapter> gripperAdapter;
    std::unique_ptr<grasplink::simulation::PhysicsSystemModule> integration;

    Fixture()
    {
        world.import<TransformSystemModule>();
        scenes.LoadScene<Scene>();
        scenes.OnUpdate(0.0F);
        scene = scenes.GetActiveScene();
        Require(scene != nullptr, "active scene exists");
        model = GltfLoader::LoadGLB(std::filesystem::path("assets") / "HCR12A_2F-85.glb");
        Require(model.rootNodeIndex >= 0, "model root exists");
        nodes.resize(model.nodes.size());
        robotRoot = CreateTree(model.rootNodeIndex, Entity{scene->GetSceneRoot()});
        gripper = robotRoot.FindChildByNameRecursive("Gripper");
        Require(static_cast<bool>(gripper), "authored Gripper exists");
        Require(controller.Connect().Ok() && controller.Activate().Ok(), "controller activated");
        armState.valid = true;
        armState.jointPositionRadians.assign(6, 0.0);
        armAdapter = std::make_unique<grasplink::viewer::robotics::RobotTransformAdapter>(robotRoot, models::hanwha::kHcr12a);
        gripperAdapter = std::make_unique<grasplink::viewer::robotics::GripperTransformAdapter>(gripper, models::robotiq::kTwoF85);
        ApplyPoses();
        grasplink::simulation::ConfigureTwoF85Colliders(*scene, robotRoot, model);
        TransformSystemModule::UpdateWorldTransforms(world);
        integration = std::make_unique<grasplink::simulation::PhysicsSystemModule>(world, physics);
    }

    Entity CreateTree(int index, const Entity& parent)
    {
        const NodeData& node = model.nodes.at(static_cast<std::size_t>(index));
        Entity entity = scene->CreateEntity(node.name);
        nodes[static_cast<std::size_t>(index)] = entity;
        entity.SetParent(parent);
        entity.SetLocalPosition(node.translation);
        entity.SetLocalRotation(node.rotation);
        entity.SetLocalScale(node.scale);
        for (int child : node.childrenIndices) CreateTree(child, entity);
        return entity;
    }

    void ApplyPoses()
    {
        armAdapter->Apply(armMath.Update(armState));
        gripperAdapter->Apply(gripperMath.Update(controller.GetState()));
        TransformSystemModule::UpdateWorldTransforms(world);
    }

    void Step(int count)
    {
        for (int step = 0; step < count; ++step)
        {
            controller.Update(0.004);
            ApplyPoses();
            integration->Step(0.004);
            TransformSystemModule::UpdateWorldTransforms(world);
        }
    }

    void Command(std::uint8_t position, std::uint8_t speed = 255)
    {
        Require(controller.Command({position, speed, 128}).Ok(), "valid gripper command accepted");
    }

    void CheckLocalPositions() const
    {
        for (std::size_t index = 0; index < nodes.size(); ++index)
            if (nodes[index] != robotRoot)
            {
                RequireVector(nodes[index].GetLocalPosition(), model.nodes[index].translation, "GLB Local position preserved");
                RequireVector(nodes[index].GetLocalScale(), model.nodes[index].scale, "GLB Local scale preserved");
            }
    }

    void CheckProxies()
    {
        std::size_t count = 0;
        const auto check = [&](const char* owner)
        {
            Entity joint = std::string(owner) == "Gripper" ? gripper : gripper.FindChildByNameRecursive(owner);
            Entity proxy = joint.GetChild(std::string(owner) + "_CollisionProxy");
            Require(static_cast<bool>(proxy), "proxy remains under owning joint");
            Require(proxy.Get<RigidBody>().motionType == BodyMotionType::Kinematic &&
                    proxy.Get<RigidBody>().collisionLayer == CollisionLayer::Gripper, "proxy motion and layer unchanged");
            RequireMatrix(proxy.GetWorldMatrix(), joint.GetWorldMatrix(), "proxy shares owning joint World pose");
            ++count;
        };
        check("Gripper");
        for (const auto& joint : models::robotiq::kTwoF85Joints) check(joint.name.data());
        Require(count == 7, "all seven gripper proxies checked");
    }

    double Gap()
    {
        // 손끝 proxy의 볼록 외피를 Gripper 기준으로 되돌려 서로 마주 보는 x 경계의 간격 [m]을 측정한다.
        const glm::mat4 inverseRoot = glm::inverse(gripper.GetWorldMatrix());
        float leftMaximum = -std::numeric_limits<float>::infinity();
        float rightMinimum = std::numeric_limits<float>::infinity();
        for (const char* side : {"Left", "Right"})
        {
            Entity proxy = gripper.FindChildByNameRecursive(std::string(side) + "FingerTipJoint_CollisionProxy");
            const glm::mat4 toRoot = inverseRoot * proxy.GetWorldMatrix();
            for (const auto& shape : proxy.Get<Colliders>().shapes)
                for (const auto& point : shape.pointsMeters)
                {
                    const float x = (toRoot * glm::vec4(point, 1.0F)).x;
                    if (std::string(side) == "Left") leftMaximum = std::max(leftMaximum, x);
                    else rightMinimum = std::min(rightMinimum, x);
                }
        }
        return rightMinimum - leftMaximum;
    }
};

void CheckBranchedPose(Fixture& fixture, const std::array<glm::vec3, 2>& outerBind,
    const std::array<glm::vec3, 2>& tipBind)
{
    const double q = fixture.controller.GetState().closureFraction * models::robotiq::kTwoF85NominalClosedMasterRadians;
    for (std::size_t side = 0; side < 2; ++side)
    {
        // GLB의 중간 노드들은 항등 bind 회전이다. 관절 원점은 바깥 관절의 -Z 회전으로 옮긴 bind offset이다.
        // tip 자신의 반대 회전은 원점 위치를 옮기지 않고 손끝 방향을 유지한다. 좌우 가지는 독립 계산한다.
        const double angle = side == 0 ? -q : q;
        const glm::vec3 offset = tipBind[side] - outerBind[side];
        const glm::vec3 expected = outerBind[side] + glm::vec3{
            static_cast<float>(std::cos(angle) * offset.x - std::sin(angle) * offset.y),
            static_cast<float>(std::sin(angle) * offset.x + std::cos(angle) * offset.y), offset.z};
        Entity tip = fixture.gripper.FindChildByNameRecursive(side == 0 ? "LeftFingerTipJoint" : "RightFingerTipJoint");
        RequireVector(InGripper(tip, fixture.gripper), expected, "nested fingertip follows only its own branch");
        const glm::mat3 tipInRoot = glm::mat3(glm::inverse(fixture.gripper.GetWorldMatrix()) * tip.GetWorldMatrix());
        const glm::quat rotation = glm::normalize(glm::quat_cast(tipInRoot));
        Require(std::abs(rotation.w) > 0.99999F, "opposite tip rotation cancels outer joint orientation");
    }
    fixture.CheckLocalPositions();
    fixture.CheckProxies();
}

void CheckNonidentityBindRotation()
{
    flecs::world world;
    SceneManager scenes(world);
    scenes.LoadScene<Scene>();
    scenes.OnUpdate(0.0F);
    Entity root = scenes.GetActiveScene()->CreateEntity("SyntheticGripper");
    Entity joint = scenes.GetActiveScene()->CreateEntity("LeftOuterKnuckleJoint");
    joint.SetParent(root);
    joint.SetLocalPosition({0.03F, 0.05F, 0.09F});
    const glm::vec3 bindEuler{0.25F, -0.4F, 0.6F};
    joint.SetLocalRotation(bindEuler);
    auto specification = models::robotiq::kTwoF85;
    specification.jointCount = 1;
    grasplink::viewer::robotics::GripperTransformAdapter adapter(root, specification);
    kinematics::GripperKinematicState pose;
    pose.masterAngleRadians = 0.4;
    pose.jointAnglesRadians = {0.4};
    pose.jointLocalRotations = {{std::cos(0.2), 0.0, 0.0, -std::sin(0.2)}};
    adapter.Apply(pose);
    const glm::quat expected = glm::quat(bindEuler) * glm::quat{
        static_cast<float>(std::cos(0.2)), 0.0F, 0.0F, static_cast<float>(-std::sin(0.2))};
    Require(std::abs(glm::dot(glm::normalize(glm::quat(joint.GetLocalRotation())), glm::normalize(expected))) > 0.99999F,
        "nonidentity bind rotation multiplies delta on the right");
    RequireVector(joint.GetLocalPosition(), {0.03F, 0.05F, 0.09F}, "nonidentity bind position untouched");
}

void CheckMotionAndHierarchy()
{
    Fixture fixture;
    const glm::mat4 mountBind = TransformSystemModule::ComposeLocalMatrix(
        fixture.gripper.GetLocalPosition(), fixture.gripper.GetLocalRotation(), fixture.gripper.GetLocalScale());
    Require(glm::length(fixture.gripper.GetLocalPosition()) > 0.09F, "GLB Gripper matrix is not mistaken for identity");
    const std::array<glm::vec3, 2> outerBind{
        InGripper(fixture.gripper.FindChildByNameRecursive("LeftOuterKnuckleJoint"), fixture.gripper),
        InGripper(fixture.gripper.FindChildByNameRecursive("RightOuterKnuckleJoint"), fixture.gripper)};
    const std::array<glm::vec3, 2> tipBind{
        InGripper(fixture.gripper.FindChildByNameRecursive("LeftFingerTipJoint"), fixture.gripper),
        InGripper(fixture.gripper.FindChildByNameRecursive("RightFingerTipJoint"), fixture.gripper)};
    CheckBranchedPose(fixture, outerBind, tipBind);
    RequireNear(fixture.Gap(), 0.085, 2.0e-5, "authored open gap is 85 mm");

    fixture.Command(255, 0);
    fixture.Step(1);
    Require(fixture.controller.GetState().actualPosition == 0 && fixture.controller.GetState().closureFraction > 0.0,
        "continuous motion precedes first 8-bit feedback increment");
    Require(std::abs(fixture.gripper.FindChildByNameRecursive("LeftOuterKnuckleJoint").GetLocalRotation().z) > 1.0e-5F,
        "continuous snapshot moves geometry without raw quantization");
    fixture.Command(128);
    fixture.Step(125);
    CheckBranchedPose(fixture, outerBind, tipBind);
    const double halfGap = fixture.Gap();
    Require(halfGap > 0.015 && halfGap < 0.06, "intermediate command narrows aperture");
    fixture.Command(255);
    fixture.Step(250);
    CheckBranchedPose(fixture, outerBind, tipBind);
    Require(fixture.Gap() >= 0.0 && fixture.Gap() < 0.003, "nominal closed geometry preserves small asset gap");
    RequireMatrix(TransformSystemModule::ComposeLocalMatrix(fixture.gripper.GetLocalPosition(),
        fixture.gripper.GetLocalRotation(), fixture.gripper.GetLocalScale()), mountBind, "nonidentity mounting matrix preserved");

    fixture.Command(0);
    fixture.Step(35);
    Require(fixture.controller.Stop().Ok(), "stop accepted during reopening");
    const double stopped = fixture.controller.GetState().closureFraction;
    fixture.Step(100);
    RequireNear(fixture.controller.GetState().closureFraction, stopped, 1.0e-12, "stop holds continuous position");
    CheckBranchedPose(fixture, outerBind, tipBind);

    // 팔과 모델 root를 함께 움직여도 그리퍼 bind와 분기 회전은 바뀌지 않고 World에만 부모 자세가 전파된다.
    fixture.armState.jointPositionRadians = {0.4, -0.2, 0.1, 0.25, -0.15, 0.3};
    fixture.robotRoot.SetLocalPosition({0.7F, 0.4F, -0.6F});
    fixture.robotRoot.SetLocalRotation({0.2F, 0.1F, -0.3F});
    fixture.Step(2);
    CheckBranchedPose(fixture, outerBind, tipBind);
    auto pose = fixture.gripperMath.Update(fixture.controller.GetState());
    std::vector<glm::vec3> before;
    for (const auto& joint : models::robotiq::kTwoF85Joints)
        before.push_back(fixture.gripper.FindChildByNameRecursive(std::string(joint.name)).GetLocalRotation());
    for (int repeat = 0; repeat < 50; ++repeat) fixture.gripperAdapter->Apply(pose);
    for (std::size_t index = 0; index < before.size(); ++index)
        RequireVector(fixture.gripper.FindChildByNameRecursive(std::string(models::robotiq::kTwoF85Joints[index].name)).GetLocalRotation(),
            before[index], "repeated apply does not accumulate rotation");

    auto bad = pose;
    bad.jointLocalRotations.back().w = std::numeric_limits<double>::quiet_NaN();
    ExpectThrows<std::invalid_argument>([&] { fixture.gripperAdapter->Apply(bad); }, "invalid late quaternion rejected atomically");
    for (std::size_t index = 0; index < before.size(); ++index)
        RequireVector(fixture.gripper.FindChildByNameRecursive(std::string(models::robotiq::kTwoF85Joints[index].name)).GetLocalRotation(),
            before[index], "invalid apply leaves every joint unchanged");
    Entity finger = fixture.gripper.FindChildByNameRecursive("LeftFinger");
    const Entity parent = finger.GetParent();
    finger.SetParent(fixture.gripper);
    ExpectThrows<std::runtime_error>([&] { fixture.gripperAdapter->Apply(pose); }, "changed intermediate ancestry rejected");
    finger.SetParent(parent);
    fixture.gripperAdapter->Apply(pose);
    fixture.scenes.LoadScene<Scene>();
    fixture.scenes.OnUpdate(0.0F);
    ExpectThrows<std::runtime_error>([&] { fixture.gripperAdapter->Apply(pose); }, "removed scene invalidates borrowed joint handles");
}

glm::vec3 SupportPoint(Entity proxy, float& top)
{
    std::vector<glm::vec3> points;
    top = -std::numeric_limits<float>::infinity();
    for (const auto& shape : proxy.Get<Colliders>().shapes)
        for (const auto& point : shape.pointsMeters)
        {
            const glm::vec3 world = glm::vec3(proxy.GetWorldMatrix() * glm::vec4(point, 1.0F));
            points.push_back(world);
            top = std::max(top, world.y);
        }
    glm::vec3 center(0.0F);
    std::size_t count = 0;
    for (const auto& point : points)
        if (top - point.y < 0.002F) { center += point; ++count; }
    Require(count != 0, "tip has gravity-facing hull surface");
    return center / static_cast<float>(count);
}

void CheckPhysicalFollowing()
{
    Fixture fixture;
    // 접촉 fixture만 Gripper를 +X 90도로 배치해 중력이 개폐 평면에 수직이게 한다.
    // 누적 팔 변환의 역변환으로 이 World 자세를 Local에 저장한다. 원본 모델 장착의 단일 축 가정은 하지 않는다.
    const glm::mat4 parentWorld = fixture.gripper.GetParent().GetWorldMatrix();
    const glm::mat4 desiredWorld = glm::translate(glm::mat4(1.0F), {0.0F, 0.5F, 0.0F}) *
        glm::mat4_cast(glm::angleAxis(glm::half_pi<float>(), glm::vec3{1.0F, 0.0F, 0.0F}));
    const glm::mat4 local = glm::inverse(parentWorld) * desiredWorld;
    fixture.gripper.SetLocalPosition(glm::vec3(local[3]));
    fixture.gripper.SetLocalRotation(glm::eulerAngles(glm::quat_cast(glm::mat3(local))));
    fixture.Command(128);
    fixture.Step(125);
    Entity tip = fixture.gripper.FindChildByNameRecursive("LeftFingerTipJoint_CollisionProxy");
    float surfaceY;
    const glm::vec3 surface = SupportPoint(tip, surfaceY);
    Entity drop = fixture.scene->CreateEntity("RuntimeTipContact");
    drop.SetLocalPosition({surface.x, surfaceY + 0.07F, surface.z});
    drop.set<RigidBody>({BodyMotionType::Dynamic, CollisionLayer::DynamicObject})
        .set<Colliders>({{physics_colliders::Box({0.003F, 0.003F, 0.003F})}});
    fixture.Step(300);
    Require(drop.GetLocalPosition().y > surfaceY + 0.001F && drop.GetLocalPosition().y < surfaceY + 0.02F,
        "Jolt contacts fingertip at controller-driven intermediate pose");
    // 이미 접촉한 물체는 마찰로 함께 움직일 수 있다. 예전 위치에서 새로 낙하시켜 지지 형상의 이동을 따로 검증한다.
    drop.Destroy();
    fixture.Command(0);
    fixture.Step(125);
    Entity oldPositionDrop = fixture.scene->CreateEntity("FormerRuntimeTipPosition");
    oldPositionDrop.SetLocalPosition({surface.x, surfaceY + 0.07F, surface.z});
    oldPositionDrop.set<RigidBody>({BodyMotionType::Dynamic, CollisionLayer::DynamicObject})
        .set<Colliders>({{physics_colliders::Box({0.003F, 0.003F, 0.003F})}});
    fixture.Step(300);
    Require(oldPositionDrop.GetLocalPosition().y < surfaceY - 0.04F, "old fingertip position loses support after reopening");
    fixture.CheckProxies();
    float openSurfaceY;
    const glm::vec3 openSurface = SupportPoint(tip, openSurfaceY);
    const auto pose = fixture.gripperMath.Update(fixture.controller.GetState());
    fixture.scenes.LoadScene<Scene>();
    fixture.scenes.OnUpdate(0.0F);
    ExpectThrows<std::runtime_error>([&] { fixture.gripperAdapter->Apply(pose); }, "scene replacement rejects stale adapter");
    Scene* replacement = fixture.scenes.GetActiveScene();
    Entity ghostCheck = replacement->CreateEntity("NoStaleGripperBody");
    ghostCheck.SetLocalPosition({openSurface.x, openSurfaceY + 0.07F, openSurface.z});
    ghostCheck.set<RigidBody>({BodyMotionType::Dynamic, CollisionLayer::DynamicObject})
        .set<Colliders>({{physics_colliders::Box({0.003F, 0.003F, 0.003F})}});
    for (int step = 0; step < 300; ++step)
    {
        TransformSystemModule::UpdateWorldTransforms(fixture.world);
        fixture.integration->Step(0.004);
    }
    Require(ghostCheck.GetLocalPosition().y < openSurfaceY - 0.2F, "scene cleanup leaves no stale gripper collision body");
}
}

/** @brief 실제 GLB의 분기 자세와 연속 개폐가 화면용 Entity·Jolt 접촉에서 일치하는지 검증한다. */
int main()
{
    try
    {
        CheckNonidentityBindRotation();
        CheckMotionAndHierarchy();
        CheckPhysicalFollowing();
        std::cout << "Gripper runtime, branched GLB poses and physical following checks passed\n";
        return 0;
    }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
