#include "scene/EntityFactory.h"

#include "Material.h"
#include "Mesh.h"

#include "components/PhysicsComponents.h"
#include "components/RenderComponents.h"
#include "robotics/models/RobotSpecification.h"
#include "scene/Scene.h"

#include <glm/glm.hpp>

#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using BodyMotionType = grasplink::physics::BodyMotionType;
const glm::vec3 kFloorColliderHalfExtentsMeters{5.0F, 0.1F, 5.0F};
const glm::vec3 kFloorColliderLocalPositionMeters{0.0F, -0.1F, 0.0F};

void ConfigureBoxPhysics(
    Entity& entity,
    BodyMotionType motionType,
    const glm::vec3& halfExtentsMeters,
    const glm::vec3& localPositionMeters = glm::vec3{0.0F})
{
    // Entity 설정: 물체 종류 + Box 크기
    // Jolt Body 생성·관리: PhysicsSystemModule
    entity.set<RigidBody>(RigidBody{motionType})
          .set<BoxCollider>(BoxCollider{halfExtentsMeters, localPositionMeters});
}
}

void EntityFactory::ConfigureFloor(Entity& floor)
{
    // Box 중심: 아래로 0.1m → 윗면 World Y=0
    // GLB Mesh: 화면 표시용
    ConfigureBoxPhysics(floor, BodyMotionType::Static,
        kFloorColliderHalfExtentsMeters, kFloorColliderLocalPositionMeters);
}

Entity EntityFactory::CreateDebugBox(Scene& scene, const std::shared_ptr<Shader>& shader)
{
    if (!shader)
        throw std::runtime_error("EntityFactory: Debug Box requires a shader");

    Entity entity = scene.CreateEntity("PhysicsDebugBox");
    entity.SetLocalPosition({0.0F, 2.0F, 0.0F});
    // Cube Mesh·Collider 한 변: 1m
    // 화면 크기와 충돌 범위 일치
    ConfigureBoxPhysics(entity, BodyMotionType::Dynamic, {0.5F, 0.5F, 0.5F});

    std::shared_ptr<Mesh> mesh = Mesh::CreateCube();
    auto material = std::make_shared<Material>(
        glm::vec4{0.95F, 0.25F, 0.05F, 1.0F}, 0.0F, 0.75F);

    entity.set<MeshFilter>(MeshFilter{mesh, 0U, 0U})
          .set<MeshRenderer>(MeshRenderer{shader, material, true});

    return entity;
}

void EntityFactory::ConfigureRobotPhysics(
    const Entity& robotRoot,
    const grasplink::robotics::models::RobotSpecification& specification)
{
    if (!robotRoot)
        throw std::runtime_error("EntityFactory: invalid robot root");

    std::vector<Entity> joints;
    joints.reserve(specification.jointCount);
    for (std::size_t i = 0; i < specification.jointCount; ++i)
    {
        const auto& jointSpec = specification.joints[i];
        Entity joint = robotRoot.FindChildByNameRecursive(std::string(jointSpec.name));
        if (!joint)
            throw std::runtime_error(
                "EntityFactory: robot joint not found: " + std::string(jointSpec.name));
        joints.push_back(joint);
    }

    for (std::size_t i = 0; i < joints.size(); ++i)
    {
        glm::vec3 linkOffset{0.0F};
        glm::vec3 linkHalfExtents{0.12F};
        if (i + 1 < joints.size())
        {
            const glm::vec3 childLocalPosition = joints[i + 1].GetLocalPosition();
            linkOffset = childLocalPosition * 0.5F;
            linkHalfExtents = glm::abs(childLocalPosition) * 0.5F + glm::vec3{0.08F};
        }

        ConfigureBoxPhysics(
            joints[i], BodyMotionType::Kinematic, linkHalfExtents, linkOffset);
    }
}
