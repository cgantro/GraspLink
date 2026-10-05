#include "simulation/robotics/GripperColliders.h"

#include "Entity.h"
#include "assets/GraphicsTypes.h"
#include "scene/Scene.h"
#include "simulation/components/PhysicsComponents.h"
#include "CollisionGeometry.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace grasplink::simulation
{
namespace
{
struct ProxySpec
{
    const char* root;
    std::array<const char*, 2> meshes;
    std::size_t meshCount;
};

// 7개 rigid part를 authored joint 프레임별로 나눈다. 손가락 양쪽이나 다음 관절의 형상은 합치지 않는다.
constexpr std::array<ProxySpec, 7> ProxySpecs{{
    {"Gripper", {"GripperMesh", nullptr}, 1},
    {"LeftOuterKnuckleJoint", {"LeftOuterKnuckleMesh", "LeftFingerMesh"}, 2},
    {"RightOuterKnuckleJoint", {"RightOuterKnuckleMesh", "RightFingerMesh"}, 2},
    {"LeftInnerKnuckleJoint", {"LeftInnerKnuckleMesh", nullptr}, 1},
    {"RightInnerKnuckleJoint", {"RightInnerKnuckleMesh", nullptr}, 1},
    {"LeftFingerTipJoint", {"LeftFingerTipMesh", nullptr}, 1},
    {"RightFingerTipJoint", {"RightFingerTipMesh", nullptr}, 1}}};

bool Finite(const glm::vec3& value)
{
    return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool IsUnitScale(const glm::vec3& scale)
{
    return Finite(scale) && std::abs(scale.x - 1.0F) <= 1.0e-5F &&
        std::abs(scale.y - 1.0F) <= 1.0e-5F && std::abs(scale.z - 1.0F) <= 1.0e-5F;
}

bool HasName(const Entity& entity, const std::string& expected)
{
    const char* name = entity.GetHandle().name();
    return name != nullptr && expected == name;
}

glm::mat4 LocalMatrix(const NodeData& node)
{
    return glm::translate(glm::mat4(1.0F), node.translation) *
        glm::mat4_cast(glm::quat(node.rotation)) * glm::scale(glm::mat4(1.0F), node.scale);
}

std::vector<glm::vec3> MeshPoints(const ModelResource& model, std::size_t meshNodeIndex,
    std::size_t rootIndex)
{
    const NodeData& node = model.nodes[meshNodeIndex];
    if (node.meshIndex < 0 || static_cast<std::size_t>(node.meshIndex) >= model.meshes.size())
        throw std::invalid_argument("TwoF85 collider mesh node has no valid mesh.");
    // 정점 [m]을 owning joint/body frame으로 옮긴다. root 자세는 runtime hierarchy가 적용한다.
    glm::mat4 meshToRoot(1.0F);
    std::size_t current = meshNodeIndex;
    while (current != rootIndex)
    {
        meshToRoot = LocalMatrix(model.nodes[current]) * meshToRoot;
        const int parent = model.nodes[current].parentIndex;
        if (parent < 0) throw std::invalid_argument("TwoF85 mesh is outside its owning joint hierarchy.");
        current = static_cast<std::size_t>(parent);
    }
    const MeshData& mesh = model.meshes[static_cast<std::size_t>(node.meshIndex)];
    std::vector<glm::vec3> vertices;
    for (const Vertex& vertex : mesh.vertices)
    {
        if (!Finite(vertex.position)) throw std::invalid_argument("TwoF85 mesh has a non-finite vertex.");
        const glm::vec4 transformed = meshToRoot * glm::vec4(vertex.position, 1.0F);
        const glm::vec3 point(transformed);
        if (!Finite(point)) throw std::invalid_argument("TwoF85 mesh transform is not finite.");
        vertices.push_back(point);
    }
    const auto support = detail::BuildConvexSupportPoints(vertices);
    if (!detail::HasHullVolume(support)) throw std::invalid_argument("TwoF85 mesh has a degenerate collision hull.");
    return support;
}
}

void ConfigureTwoF85Colliders(Scene& scene, const Entity& robotRoot, const ModelResource& model)
{
    if (!robotRoot) throw std::invalid_argument("TwoF85 collider robot root is invalid.");
    const flecs::entity sceneRoot = scene.GetSceneRoot();
    bool belongsToScene = false;
    for (Entity current = robotRoot; current; current = current.GetParent())
        if (current.GetHandle() == sceneRoot) { belongsToScene = true; break; }
    if (!belongsToScene) throw std::invalid_argument("TwoF85 robot root does not belong to the supplied scene.");
    for (Entity current = robotRoot; current && current.GetHandle() != sceneRoot; current = current.GetParent())
        if (!IsUnitScale(current.GetLocalScale()))
            throw std::invalid_argument("TwoF85 scene ancestry has non-unit scale.");
    std::unordered_map<std::string, std::size_t> modelIndices;
    std::unordered_set<std::string> expectedNodes;
    for (const ProxySpec& spec : ProxySpecs)
    {
        expectedNodes.emplace(spec.root);
        for (std::size_t i = 0; i < spec.meshCount; ++i) expectedNodes.emplace(spec.meshes[i]);
    }
    for (std::size_t i = 0; i < model.nodes.size(); ++i)
        if (expectedNodes.count(model.nodes[i].name) != 0 && !modelIndices.emplace(model.nodes[i].name, i).second)
            throw std::invalid_argument("TwoF85 model contains duplicate node names.");

    // Local T * quat(Euler) * S를 기준으로 parent index와 cycle을 모델 전체에서 검사한다.
    // 단위 scale 제한은 아래에서 실제 collider ancestry에만 적용한다.
    std::vector<unsigned char> state(model.nodes.size());
    auto build = [&](auto&& self, std::size_t i) -> void
    {
        if (state[i] == 1) throw std::invalid_argument("TwoF85 model contains a node cycle.");
        if (state[i] == 2) return;
        state[i] = 1;
        const NodeData& node = model.nodes[i];
        if (node.parentIndex < -1)
            throw std::invalid_argument("TwoF85 model has an invalid parent index.");
        if (node.parentIndex >= 0)
        {
            if (static_cast<std::size_t>(node.parentIndex) >= model.nodes.size())
                throw std::invalid_argument("TwoF85 model has an invalid parent index.");
            self(self, static_cast<std::size_t>(node.parentIndex));
        }
        state[i] = 2;
    };
    for (std::size_t i = 0; i < model.nodes.size(); ++i) build(build, i);

    auto validateAncestry = [&](std::size_t index)
    {
        while (true)
        {
            const NodeData& node = model.nodes[index];
            if (!Finite(node.translation) || !Finite(node.rotation) || !Finite(node.scale))
                throw std::invalid_argument("TwoF85 collider ancestry has non-finite TRS.");
            if (!IsUnitScale(node.scale))
                throw std::invalid_argument("TwoF85 collider ancestry has non-unit scale.");
            if (node.parentIndex < 0) break;
            index = static_cast<std::size_t>(node.parentIndex);
        }
    };

    struct Prepared { Entity parent; std::vector<physics::CollisionShapeDescription> shapes; std::string name; };
    std::array<Prepared, ProxySpecs.size()> prepared;
    for (std::size_t p = 0; p < ProxySpecs.size(); ++p)
    {
        const ProxySpec& spec = ProxySpecs[p];
        const auto rootIt = modelIndices.find(spec.root);
        if (rootIt == modelIndices.end()) throw std::invalid_argument("TwoF85 required model node is missing.");
        const std::size_t rootIndex = rootIt->second;
        validateAncestry(rootIndex);
        Entity rootEntity = robotRoot.FindChildByNameRecursive(spec.root);
        if (!rootEntity) throw std::invalid_argument("TwoF85 required scene entity is missing.");
        Entity liveAncestor = rootEntity;
        while (liveAncestor && liveAncestor != robotRoot)
        {
            if (!IsUnitScale(liveAncestor.GetLocalScale()))
                throw std::invalid_argument("TwoF85 scene joint ancestry has non-unit scale.");
            liveAncestor = liveAncestor.GetParent();
        }
        if (liveAncestor != robotRoot)
            throw std::invalid_argument("TwoF85 joint is outside the robot root hierarchy.");
        const std::size_t gripperIndex = modelIndices.at("Gripper");
        std::size_t modelAncestor = rootIndex;
        Entity sceneAncestor = rootEntity;
        while (modelAncestor != gripperIndex)
        {
            if (!IsUnitScale(sceneAncestor.GetLocalScale()))
                throw std::invalid_argument("TwoF85 scene joint ancestry has non-unit scale.");
            const int parentIndex = model.nodes[modelAncestor].parentIndex;
            if (parentIndex < 0 || !sceneAncestor)
                throw std::invalid_argument("TwoF85 joint is outside the Gripper hierarchy.");
            modelAncestor = static_cast<std::size_t>(parentIndex);
            sceneAncestor = sceneAncestor.GetParent();
            if (!sceneAncestor || !HasName(sceneAncestor, model.nodes[modelAncestor].name))
                throw std::invalid_argument("TwoF85 scene joint hierarchy does not match the model.");
        }
        prepared[p].parent = rootEntity;
        prepared[p].name = std::string(spec.root) + "_CollisionProxy";
        if (robotRoot.FindChildByNameRecursive(prepared[p].name))
            throw std::invalid_argument("TwoF85 collision proxy already exists.");

        for (std::size_t m = 0; m < spec.meshCount; ++m)
        {
            const auto meshIt = modelIndices.find(spec.meshes[m]);
            if (meshIt == modelIndices.end()) throw std::invalid_argument("TwoF85 required mesh node is missing.");
            const std::size_t meshIndex = meshIt->second;
            validateAncestry(meshIndex);
            std::size_t ancestor = meshIndex;
            while (ancestor != rootIndex && model.nodes[ancestor].parentIndex >= 0)
                ancestor = static_cast<std::size_t>(model.nodes[ancestor].parentIndex);
            if (ancestor != rootIndex) throw std::invalid_argument("TwoF85 mesh is outside its owning joint hierarchy.");
            Entity meshEntity = robotRoot.FindChildByNameRecursive(spec.meshes[m]);
            if (!meshEntity) throw std::invalid_argument("TwoF85 required scene mesh entity is missing.");
            // Prefab의 부모 체인도 GLB NodeData와 일치해야 다른 관절 형상을 잘못 가져오지 않는다.
            Entity sceneAncestor = meshEntity;
            std::size_t modelMeshAncestor = meshIndex;
            while (true)
            {
                if (!sceneAncestor || !HasName(sceneAncestor, model.nodes[modelMeshAncestor].name))
                    throw std::invalid_argument("TwoF85 scene mesh hierarchy does not match the model.");
                if (!IsUnitScale(sceneAncestor.GetLocalScale()))
                    throw std::invalid_argument("TwoF85 scene mesh ancestry has non-unit scale.");
                if (modelMeshAncestor == rootIndex) break;
                const int parentIndex = model.nodes[modelMeshAncestor].parentIndex;
                if (parentIndex < 0) throw std::invalid_argument("TwoF85 mesh is outside its owning joint hierarchy.");
                modelMeshAncestor = static_cast<std::size_t>(parentIndex);
                sceneAncestor = sceneAncestor.GetParent();
            }
            physics::CollisionShapeDescription shape;
            shape.type = physics::CollisionShapeType::ConvexHull;
            shape.pointsMeters = MeshPoints(model, meshIndex, rootIndex);
            prepared[p].shapes.push_back(std::move(shape));
        }
    }

    // proxy는 authored joint의 자식이며 local pose identity로 둔다. 관절 Local 변경이 proxy에 전파된다.
    // gripper backend가 연결되기 전까지 원본 ECS joint hierarchy가 7 rigid part의 자세를 제공한다.
    // Fixed Update가 World transform을 갱신한 뒤 PhysicsSystemModule이 Kinematic 목표 자세를 동기화한다.
    for (Prepared& item : prepared)
    {
        Entity proxy = scene.CreateEntity(item.name);
        proxy.SetParent(item.parent);
        proxy.SetLocalPosition(glm::vec3(0.0F));
        proxy.SetLocalRotation(glm::vec3(0.0F));
        proxy.SetLocalScale(glm::vec3(1.0F));
        proxy.set<RigidBody>(RigidBody{physics::BodyMotionType::Kinematic, physics::CollisionLayer::Gripper})
            .set<Colliders>(Colliders{std::move(item.shapes)});
    }
}
}
