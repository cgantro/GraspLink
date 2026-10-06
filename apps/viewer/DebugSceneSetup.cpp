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

// 아래 값은 physicsDemo에서 상자를 떨어뜨려 보는 장면의 배치다. 화면 Mesh와 Box 충돌 형상은 모두 한 변 12 cm로 맞춘다.
void CreatePhysicsBoxes(Scene& scene, const std::shared_ptr<Shader>& shader)
{
    if (!shader)
        throw std::runtime_error("DebugSceneSetup: Physics Boxes require a shader");

    std::shared_ptr<Mesh> mesh = Mesh::CreateCube(kDebugBoxSizeMeters);
    const auto material = std::make_shared<Material>(glm::vec4{0.95F, 0.25F, 0.05F, 1.0F}, 0.0F, 0.75F);
    const glm::vec3 halfExtents{kDebugBoxSizeMeters * 0.5F};
    const std::size_t layers = kDebugBoxCount / (kDebugBoxColumns * kDebugBoxRows);
    std::size_t index = 0;

    // 상자를 가로 5개, 깊이 2개, 높이 5개로 배치한다. 위치는 SceneRoot 기준 [m]이고 Mesh와 Collider 중심은 Entity 원점에 둔다. 낙하와 접촉은 Jolt가 계산한다.
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
                // 여기서는 Flecs에 움직이는 Body 종류와 충돌 형상 설정만 붙인다.
                // PhysicsSystemModule이 Jolt Body를 만들고 매 고정 시간 갱신 결과를 Entity 위치에 되돌린다.
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
    // 이 함수는 첫 원소에 접근하므로 모델 사양에 관절이 하나 이상 있어야 한다. 이 데모는 첫 관절의 기준 위치로부터 30도인 절대 목표각을 지정한다.
    grasplink::robotics::JointMoveCommand command;
    command.targetPositionRadians.assign(specification.jointCount, 0.0);
    command.targetPositionRadians[0] = glm::radians(30.0);
    command.velocityScale = 1.0;
    command.accelerationScale = 1.0;
    return controller.MoveJoint(command);
}
}
