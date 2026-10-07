#include "PickPlaceScenario.h"

#include "Material.h"
#include "Mesh.h"
#include "components/RenderComponents.h"
#include "scene/Scene.h"
#include "simulation/components/PhysicsComponents.h"
#include "simulation/RandomScenario.h"

#include <glm/glm.hpp>

#include <stdexcept>

namespace grasplink::viewer::pick_place
{
Entity CreateGraspBox(Scene& scene, const std::shared_ptr<Shader>& shader)
{
    if (!shader)
        throw std::runtime_error("PickPlaceScenario: grasp box requires a shader");
    constexpr float sideMeters = 0.04F;
    Entity box = scene.CreateEntity("GraspBox");
    // 바닥의 물리 윗면은 Y=0.005 m이므로 중심을 0.026 m에 두어 상자 아랫면과 바닥 사이에 1 mm의 초기 간격을 둔다.
    // 시작 자세에서 반복 검증한 작업 공간 안에서 매 실행 상자를 새 위치에 놓는다.
    // 넓힌 무작위 범위도 작업대 평면과 로봇의 기본 도달 거리 안에 둔다.
    const auto position = grasplink::simulation::scenario::SampleBoxPosition();
    box.SetLocalPosition({position[0], position[1], position[2]});
    box.SetLocalRotation(glm::angleAxis(
        grasplink::simulation::scenario::SamplePlanarRotation(), glm::vec3{0.0F, 1.0F, 0.0F}));
    box.set<RigidBody>({grasplink::physics::BodyMotionType::Dynamic, grasplink::physics::CollisionLayer::DynamicObject})
        .set<Colliders>({{physics_colliders::Box(glm::vec3(sideMeters * 0.5F))}});
    box.set<MeshFilter>({Mesh::CreateCube(sideMeters), 0U, 0U})
        .set<MeshRenderer>({shader, std::make_shared<Material>(glm::vec4{0.15F, 0.65F, 0.95F, 1.0F}, 0.0F, 0.75F), true});
    return box;
}

Entity CreatePlacementArea(Scene& scene, const std::shared_ptr<Shader>& shader)
{
    if (!shader)
        throw std::runtime_error("PickPlaceScenario: placement area requires a shader");
    constexpr float boxSideMeters = 0.04F;
    constexpr float areaSideMeters = boxSideMeters * 1.7320508F;
    constexpr float areaThicknessMeters = 0.002F;
    Entity area = scene.CreateEntity("PlacementArea");
    const auto position = grasplink::simulation::scenario::SamplePlacementPosition();
    area.SetLocalPosition({position[0], position[1], position[2]});
    area.SetLocalRotation(glm::angleAxis(
        grasplink::simulation::scenario::SamplePlanarRotation(), glm::vec3{0.0F, 1.0F, 0.0F}));
    area.SetLocalScale({areaSideMeters, areaThicknessMeters, areaSideMeters});
    area.set<MeshFilter>({Mesh::CreateCube(1.0F), 0U, 0U})
        .set<MeshRenderer>({shader,
            std::make_shared<Material>(glm::vec4{0.18F, 0.82F, 0.32F, 1.0F}, 0.0F, 0.8F), true});
    return area;
}

} // namespace grasplink::viewer::pick_place
