#include "DebugSceneSetup.h"

#include "Material.h"
#include "Mesh.h"
#include "components/RenderComponents.h"
#include "robotics/core/IRobotController.h"
#include "robotics/models/RobotSpecification.h"
#include "scene/Scene.h"
#include "simulation/components/PhysicsComponents.h"

#include <glm/glm.hpp>

#include <stdexcept>
#include <string>

namespace viewer_debug
{
namespace
{
constexpr std::size_t kDebugBoxCount = 50;
constexpr std::size_t kDebugBoxColumns = 5;
constexpr std::size_t kDebugBoxRows = 2;
constexpr float kDebugBoxSizeMeters = 0.12F;
constexpr float kDebugBoxSpacingMeters = 0.14F;
constexpr float kDebugBoxStartHeightMeters = 1.65F;
constexpr float kDebugBoxCenterZMeters = 0.48F;
}

void CreatePhysicsBoxes(Scene& scene, const std::shared_ptr<Shader>& shader)
{
    if (!shader)
        throw std::runtime_error("DebugSceneSetup: Physics Boxes require a shader");

    std::shared_ptr<Mesh> mesh = Mesh::CreateCube(kDebugBoxSizeMeters);
    const auto material = std::make_shared<Material>(glm::vec4{0.95F, 0.25F, 0.05F, 1.0F}, 0.0F, 0.75F);
    const glm::vec3 halfExtents{kDebugBoxSizeMeters * 0.5F};
    const std::size_t layers = kDebugBoxCount / (kDebugBoxColumns * kDebugBoxRows);
    std::size_t index = 0;

    // 5×2×5 배열. SceneRoot 기준 [m]으로 배치하고, 낙하는 Jolt가 계산한다.
    for (std::size_t layer = 0; layer < layers; ++layer)
    {
        for (std::size_t row = 0; row < kDebugBoxRows; ++row)
        {
            for (std::size_t column = 0; column < kDebugBoxColumns; ++column)
            {
                Entity entity = scene.CreateEntity("PhysicsDebugBox_" + std::to_string(index++));
                entity.SetLocalPosition({-0.28F + static_cast<float>(column) * kDebugBoxSpacingMeters,
                    kDebugBoxStartHeightMeters + static_cast<float>(layer) * kDebugBoxSpacingMeters,
                    kDebugBoxCenterZMeters + static_cast<float>(row) * kDebugBoxSpacingMeters});
                // ECS 설정만 붙인다. Body 생성과 Entity 위치 반영은 PhysicsSystemModule의 책임이다.
                entity.set<RigidBody>(RigidBody{
                    grasplink::physics::BodyMotionType::Dynamic, grasplink::physics::CollisionLayer::DynamicObject})
                    .set<Colliders>(Colliders{{physics_colliders::Box(halfExtents)}});
                entity.set<MeshFilter>(MeshFilter{mesh, 0U, 0U})
                    .set<MeshRenderer>(MeshRenderer{shader, material, true});
            }
        }
    }
}

grasplink::robotics::Result StartRobotMotion(grasplink::robotics::IRobotController& controller,
    const grasplink::robotics::models::RobotSpecification& specification)
{
    grasplink::robotics::JointMoveCommand command;
    command.targetPositionRadians.assign(specification.jointCount, 0.0);
    command.targetPositionRadians[0] = glm::radians(30.0);
    command.velocityScale = 1.0;
    command.accelerationScale = 1.0;
    return controller.MoveJoint(command);
}
}
