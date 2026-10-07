#include "DebugSceneSetup.h"

#include "Material.h"
#include "Mesh.h"
#include "components/RenderComponents.h"
#include "robotics/core/IRobotController.h"
#include "robotics/models/RobotSpecification.h"
#include "scene/Scene.h"
#include "simulation/components/PhysicsComponents.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <stdexcept>
#include <random>
#include <string>
#include <cmath>

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

std::mt19937& RandomGenerator()
{
    static std::mt19937 random{std::random_device{}()};
    return random;
}

std::array<float, 3> SamplePlanarPosition(float heightMeters, float minimumRadiusMeters,
    float maximumRadiusMeters, float minimumSeparationMeters, std::array<float, 3>& previousPosition,
    bool& hasPreviousPosition)
{
    std::uniform_real_distribution<float> xPosition{-1.05F, 1.05F};
    std::uniform_real_distribution<float> zPosition{-1.05F, 1.05F};
    std::array<float, 3> position{};
    float radius = 0.0F;

    do
    {
        position = {xPosition(RandomGenerator()), heightMeters, zPosition(RandomGenerator())};
        radius = std::hypot(position[0], position[2]);
    } while (radius < minimumRadiusMeters || radius > maximumRadiusMeters ||
        (hasPreviousPosition && std::hypot(position[0] - previousPosition[0],
            position[2] - previousPosition[2]) < minimumSeparationMeters));

    previousPosition = position;
    hasPreviousPosition = true;
    return position;
}
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

Entity CreateGraspBox(Scene& scene, const std::shared_ptr<Shader>& shader)
{
    if (!shader)
        throw std::runtime_error("DebugSceneSetup: grasp box requires a shader");
    constexpr float sideMeters = 0.04F;
    Entity box = scene.CreateEntity("GraspBox");
    // 바닥의 물리 윗면은 Y=0.005 m이므로 중심을 0.026 m에 두어 상자 아랫면과 바닥 사이에 1 mm의 초기 간격을 둔다.
    // 시작 자세에서 반복 검증한 작업 공간 안에서 매 실행 상자를 새 위치에 놓는다.
    // 넓힌 무작위 범위도 작업대 평면과 로봇의 기본 도달 거리 안에 둔다.
    const auto position = RandomGraspBoxPosition();
    box.SetLocalPosition({position[0], position[1], position[2]});
    box.SetLocalRotation(glm::angleAxis(RandomPlanarRotationRadians(), glm::vec3{0.0F, 1.0F, 0.0F}));
    box.set<RigidBody>({grasplink::physics::BodyMotionType::Dynamic, grasplink::physics::CollisionLayer::DynamicObject})
        .set<Colliders>({{physics_colliders::Box(glm::vec3(sideMeters * 0.5F))}});
    box.set<MeshFilter>({Mesh::CreateCube(sideMeters), 0U, 0U})
        .set<MeshRenderer>({shader, std::make_shared<Material>(glm::vec4{0.15F, 0.65F, 0.95F, 1.0F}, 0.0F, 0.75F), true});
    return box;
}

std::array<float, 3> RandomGraspBoxPosition()
{
    static std::array<float, 3> previous{};
    static bool hasPrevious = false;
    // 이전 상자와 35 mm보다 가까운 표본은 다시 뽑아 재배치가 눈에 띄게 한다.
    return SamplePlanarPosition(0.026F, 0.50F, 1.05F, 0.035F, previous, hasPrevious);
}

std::array<float, 3> RandomPlacementAreaPosition()
{
    static std::array<float, 3> previous{};
    static bool hasPrevious = false;
    // 이전 목표판과 50 mm보다 가까운 표본은 다시 뽑아 새 목표가 분명히 달라지게 한다.
    return SamplePlanarPosition(0.006F, 0.50F, 1.00F, 0.05F, previous, hasPrevious);
}

float RandomPlanarRotationRadians()
{
    std::uniform_real_distribution<float> angle{0.0F, 6.28318530718F};
    return angle(RandomGenerator());
}

Entity CreatePlacementArea(Scene& scene, const std::shared_ptr<Shader>& shader)
{
    if (!shader)
        throw std::runtime_error("DebugSceneSetup: placement area requires a shader");
    constexpr float boxSideMeters = 0.04F;
    constexpr float areaSideMeters = boxSideMeters * 1.7320508F;
    constexpr float areaThicknessMeters = 0.002F;
    Entity area = scene.CreateEntity("PlacementArea");
    const auto position = RandomPlacementAreaPosition();
    area.SetLocalPosition({position[0], position[1], position[2]});
    area.SetLocalRotation(glm::angleAxis(RandomPlanarRotationRadians(), glm::vec3{0.0F, 1.0F, 0.0F}));
    area.SetLocalScale({areaSideMeters, areaThicknessMeters, areaSideMeters});
    area.set<MeshFilter>({Mesh::CreateCube(1.0F), 0U, 0U})
        .set<MeshRenderer>({shader,
            std::make_shared<Material>(glm::vec4{0.18F, 0.82F, 0.32F, 1.0F}, 0.0F, 0.8F), true});
    return area;
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
