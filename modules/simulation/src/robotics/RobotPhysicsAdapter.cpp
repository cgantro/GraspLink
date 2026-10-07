#include "simulation/robotics/RobotPhysicsAdapter.h"
#include "simulation/robotics/RobotCollisionGeometryBuilder.h"

#include "robotics/kinematics/RobotKinematics.h"
#include "scene/Scene.h"
#include "simulation/components/PhysicsComponents.h"
#include "simulation/components/RobotCollisionProxy.h"

#include <glm/gtx/quaternion.hpp>

#include <stdexcept>
#include <string>
#include <utility>

namespace grasplink::simulation
{
RobotPhysicsAdapter::RobotPhysicsAdapter(
    Scene& scene,
    const Entity& robotRoot,
    const grasplink::robotics::models::RobotSpecification& specification,
    const ModelResource& model)
{
    if (!robotRoot) throw std::runtime_error("RobotPhysicsAdapter: invalid robot root");
    auto geometry = robot_collision_geometry::Build(specification, model);
    if (!geometry.baseShapes.empty())
    {
        base_ = scene.CreateEntity("Base_CollisionProxy");
        base_.SetParent(robotRoot);
        base_.Add<RobotCollisionProxy>();
        base_.Add<RobotBaseEnvironmentProxy>();
        base_.set<RigidBody>(RigidBody{
                grasplink::physics::BodyMotionType::Static,
                grasplink::physics::CollisionLayer::Environment})
            .set<Colliders>(Colliders{std::move(geometry.baseShapes)});
    }

    links_.reserve(geometry.links.size());
    for (std::size_t i = 0; i < geometry.links.size(); ++i)
    {
        const auto& link = specification.links[i];
        auto& linkGeometry = geometry.links[i];
        Entity entity = scene.CreateEntity(std::string(link.name) + "_CollisionProxy");
        // FK에서 나온 위치와 회전은 Robot base 기준이다. proxy를 robotRoot의 자식으로 두어 Scene에서 로봇에 설정한 배치 변환이 부모 계층을 통해 추가된다.
        entity.SetParent(robotRoot);
        entity.Add<RobotCollisionProxy>()
            .set<RigidBody>(RigidBody{
                grasplink::physics::BodyMotionType::Kinematic,
                grasplink::physics::CollisionLayer::Robot})
            .set<Colliders>(Colliders{std::move(linkGeometry.shapes)});
        links_.push_back({entity, linkGeometry.jointIndex});
    }
}

void RobotPhysicsAdapter::Apply(
    const grasplink::robotics::kinematics::RobotKinematicState& state)
{
    for (LinkBinding& link : links_)
    {
        if (!link.entity)
            throw std::runtime_error("RobotPhysicsAdapter: robot Scene has been removed");
        if (link.jointIndex >= state.linkPosesInBaseFrame.size())
            throw std::invalid_argument("RobotPhysicsAdapter: LinkPose count mismatch");
        const auto& pose = state.linkPosesInBaseFrame[link.jointIndex];
        link.entity.SetLocalPosition({
            static_cast<float>(pose.positionMeters.x),
            static_cast<float>(pose.positionMeters.y),
            static_cast<float>(pose.positionMeters.z)});
        // base 기준 FK quaternion을 double 정밀도로 정규화한 뒤 Entity의 float quaternion에 저장한다.
        link.entity.SetLocalRotation(glm::quat{glm::normalize(glm::dquat{
            pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z})});
    }
}
}
