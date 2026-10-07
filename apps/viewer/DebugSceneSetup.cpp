#include "DebugSceneSetup.h"

#include "Material.h"
#include "Mesh.h"
#include "components/RenderComponents.h"
#include "scene/Scene.h"
#include "simulation/components/PhysicsComponents.h"

#include <glm/glm.hpp>

#include <stdexcept>
#include <random>
#include <cmath>

namespace viewer_debug
{
namespace
{
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
    std::uniform_real_distribution<float> angle{-1.57079632679F, 1.57079632679F};
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


}
