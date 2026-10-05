#include "simulation/robotics/RobotPhysicsAdapter.h"
#include "CollisionGeometry.h"

#include "robotics/kinematics/RobotKinematics.h"
#include "scene/Scene.h"
#include "simulation/components/PhysicsComponents.h"
#include "simulation/components/RobotCollisionProxy.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <numeric>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace grasplink::simulation
{
namespace
{
glm::mat4 NodeLocalTransform(const NodeData& node)
{
    return glm::translate(glm::mat4(1.0F), node.translation) *
        glm::mat4_cast(glm::quat(node.rotation)) * glm::scale(glm::mat4(1.0F), node.scale);
}

std::vector<glm::mat4> BuildNodeWorldTransforms(const ModelResource& model)
{
    // GLB 모델 내부의 누적 변환. Scene World 변환은 robotRoot에서 나중에 적용한다.
    std::vector<glm::mat4> transforms(model.nodes.size(), glm::mat4(1.0F));
    std::vector<bool> ready(model.nodes.size(), false);
    std::vector<bool> visiting(model.nodes.size(), false);
    auto build = [&](auto&& self, std::size_t index) -> const glm::mat4&
    {
        if (ready[index]) return transforms[index];
        if (visiting[index])
            throw std::invalid_argument("Robot collision model contains a node cycle.");
        visiting[index] = true;
        const NodeData& node = model.nodes[index];
        transforms[index] = NodeLocalTransform(node);
        if (node.parentIndex >= 0)
        {
            if (node.parentIndex >= static_cast<int>(model.nodes.size()))
                throw std::runtime_error("Robot collision model has an invalid parent node.");
            transforms[index] = self(self, static_cast<std::size_t>(node.parentIndex)) * transforms[index];
        }
        ready[index] = true;
        visiting[index] = false;
        return transforms[index];
    };
    for (std::size_t i = 0; i < model.nodes.size(); ++i) build(build, i);
    return transforms;
}

struct PositionKey
{
    std::int64_t x;
    std::int64_t y;
    std::int64_t z;

    bool operator==(const PositionKey& other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct PositionKeyHash
{
    std::size_t operator()(const PositionKey& key) const
    {
        std::size_t result = std::hash<std::int64_t>{}(key.x);
        result ^= std::hash<std::int64_t>{}(key.y) + 0x9e3779b9U + (result << 6U) + (result >> 2U);
        result ^= std::hash<std::int64_t>{}(key.z) + 0x9e3779b9U + (result << 6U) + (result >> 2U);
        return result;
    }
};

PositionKey MakePositionKey(const glm::vec3& position)
{
    // 약 0.01 mm 단위로 같은 위치를 묶어 Mesh의 중복 정점도 연결한다.
    constexpr double Precision = 100000.0;
    return {
        static_cast<std::int64_t>(std::llround(position.x * Precision)),
        static_cast<std::int64_t>(std::llround(position.y * Precision)),
        static_cast<std::int64_t>(std::llround(position.z * Precision))};
}

PositionKey MakeCollisionCellKey(const glm::vec3& position)
{
    // 단위: 한 셀은 16 cm. 긴 링크의 분할 수를 줄이되 부품 사이 빈 공간은 합치지 않는다.
    constexpr float CellSizeMeters = 0.16F;
    return {
        static_cast<std::int64_t>(std::floor(position.x / CellSizeMeters)),
        static_cast<std::int64_t>(std::floor(position.y / CellSizeMeters)),
        static_cast<std::int64_t>(std::floor(position.z / CellSizeMeters))};
}

std::size_t FindRoot(std::vector<std::size_t>& parents, std::size_t index)
{
    if (parents[index] != index)
        parents[index] = FindRoot(parents, parents[index]);
    return parents[index];
}

void Join(std::vector<std::size_t>& parents, std::size_t left, std::size_t right)
{
    const std::size_t leftRoot = FindRoot(parents, left);
    const std::size_t rightRoot = FindRoot(parents, right);
    if (leftRoot != rightRoot) parents[rightRoot] = leftRoot;
}

bool HasExtent(const std::vector<glm::vec3>& vertices, float minimumMeters)
{
    if (vertices.empty()) return false;
    glm::vec3 minimum = vertices.front();
    glm::vec3 maximum = vertices.front();
    for (const glm::vec3& point : vertices)
    {
        minimum = glm::min(minimum, point);
        maximum = glm::max(maximum, point);
    }
    const glm::vec3 extent = maximum - minimum;
    return std::max(extent.x, std::max(extent.y, extent.z)) >= minimumMeters;
}

std::vector<grasplink::physics::CollisionShapeDescription> BuildLinkShapes(
    const ModelResource& model,
    const std::vector<glm::mat4>& nodeTransforms,
    const NodeData& jointNode,
    const NodeData& linkNode,
    const std::unordered_set<std::string>& movingNodes)
{
    using Shape = grasplink::physics::CollisionShapeDescription;
    std::vector<Shape> shapes;
    const auto jointIterator = std::find_if(model.nodes.begin(), model.nodes.end(),
        [&](const NodeData& node) { return &node == &jointNode; });
    const auto linkIterator = std::find_if(model.nodes.begin(), model.nodes.end(),
        [&](const NodeData& node) { return &node == &linkNode; });
    if (jointIterator == model.nodes.end() || linkIterator == model.nodes.end()) return shapes;

    const std::size_t jointIndex = static_cast<std::size_t>(jointIterator - model.nodes.begin());
    const std::size_t linkIndex = static_cast<std::size_t>(linkIterator - model.nodes.begin());
    // Collider 정점은 이 link를 움직이는 관절 원점 기준으로 저장한다.
    const glm::mat4 jointInverse = glm::inverse(nodeTransforms[jointIndex]);

    for (std::size_t nodeIndex = 0; nodeIndex < model.nodes.size(); ++nodeIndex)
    {
        std::size_t ancestor = nodeIndex;
        bool insideLink = false;
        while (true)
        {
            if (ancestor == linkIndex)
            {
                insideLink = true;
                break;
            }
            // 다음 가동 관절 아래는 다른 link 소유. Gripper도 이 adapter의 충돌 범위에서 제외한다.
            if (movingNodes.count(model.nodes[ancestor].name) != 0 ||
                model.nodes[ancestor].name == "Gripper")
                break;
            if (model.nodes[ancestor].parentIndex < 0)
                break;
            ancestor = static_cast<std::size_t>(model.nodes[ancestor].parentIndex);
        }
        if (!insideLink) continue;

        const NodeData& node = model.nodes[nodeIndex];
        if (node.meshIndex < 0 || node.meshIndex >= static_cast<int>(model.meshes.size())) continue;
        const MeshData& mesh = model.meshes[static_cast<std::size_t>(node.meshIndex)];
        const glm::mat4 meshToJoint = jointInverse * nodeTransforms[nodeIndex];

        for (const SubMeshInfo& subMesh : mesh.subMeshes)
        {
            const std::size_t end = static_cast<std::size_t>(subMesh.indexStart) + subMesh.indexCount;
            if (end > mesh.indices.size()) throw std::runtime_error("Robot GLB has an invalid primitive index range.");
            std::vector<std::size_t> parents(mesh.vertices.size());
            std::iota(parents.begin(), parents.end(), 0U);
            std::unordered_set<std::uint32_t> usedVertices;
            std::unordered_map<PositionKey, std::uint32_t, PositionKeyHash> matchingPositions;
            usedVertices.reserve(subMesh.indexCount);
            matchingPositions.reserve(subMesh.indexCount);

            for (std::size_t index = subMesh.indexStart; index < end; ++index)
            {
                const std::uint32_t vertexIndex = mesh.indices[index];
                if (vertexIndex >= mesh.vertices.size()) throw std::runtime_error("Robot GLB has an invalid vertex index.");
                usedVertices.insert(vertexIndex);
                const auto [iterator, inserted] = matchingPositions.emplace(
                    MakePositionKey(mesh.vertices[vertexIndex].position), vertexIndex);
                if (!inserted) Join(parents, vertexIndex, iterator->second);
            }

            // 겹친 정점과 삼각형 연결로 부품을 나눠 떨어진 부품 사이를 hull로 잇지 않는다.
            for (std::size_t index = subMesh.indexStart; index + 2 < end; index += 3)
            {
                Join(parents, mesh.indices[index], mesh.indices[index + 1]);
                Join(parents, mesh.indices[index], mesh.indices[index + 2]);
            }

            std::unordered_map<std::size_t, std::vector<glm::vec3>> components;
            for (const std::uint32_t vertexIndex : usedVertices)
            {
                const glm::vec4 point = meshToJoint * glm::vec4(mesh.vertices[vertexIndex].position, 1.0F);
                components[FindRoot(parents, vertexIndex)].emplace_back(point);
            }

            std::unordered_map<std::size_t,
                std::unordered_map<PositionKey, std::vector<glm::vec3>, PositionKeyHash>> collisionCells;
            // 삼각형 중심을 16 cm 셀로 묶는다. 삼각형을 자르지는 않으며 셀별 hull로 근사한다.
            for (std::size_t index = subMesh.indexStart; index + 2 < end; index += 3)
            {
                const std::uint32_t first = mesh.indices[index];
                const std::size_t component = FindRoot(parents, first);
                const glm::vec3 a = glm::vec3(meshToJoint * glm::vec4(mesh.vertices[first].position, 1.0F));
                const glm::vec3 b = glm::vec3(meshToJoint * glm::vec4(mesh.vertices[mesh.indices[index + 1]].position, 1.0F));
                const glm::vec3 c = glm::vec3(meshToJoint * glm::vec4(mesh.vertices[mesh.indices[index + 2]].position, 1.0F));
                const PositionKey cell = MakeCollisionCellKey((a + b + c) / 3.0F);
                auto& points = collisionCells[component][cell];
                points.insert(points.end(), {a, b, c});
            }

            for (const auto& [component, cells] : collisionCells)
            {
                // 작은 장식 부품(<4 cm)과 셀(<1 cm)은 제외한다. 시각 Mesh와 충돌 형상은 다를 수 있다.
                if (!HasExtent(components[component], 0.04F)) continue;
                for (const auto& [cell, vertices] : cells)
                {
                    (void)cell;
                    if (!HasExtent(vertices, 0.01F)) continue;
                    std::vector<glm::vec3> uniqueVertices;
                    std::unordered_set<PositionKey, PositionKeyHash> uniquePositions;
                    uniqueVertices.reserve(vertices.size());
                    uniquePositions.reserve(vertices.size());
                    for (const glm::vec3& vertex : vertices)
                    {
                        if (uniquePositions.insert(MakePositionKey(vertex)).second)
                            uniqueVertices.push_back(vertex);
                    }
                    Shape shape;
                    shape.type = grasplink::physics::CollisionShapeType::ConvexHull;
                    shape.pointsMeters = detail::BuildConvexSupportPoints(uniqueVertices);
                    // 극점을 추린 뒤에도 두께가 남은 셀만 충돌 형상에 넣는다.
                    if (detail::HasHullVolume(shape.pointsMeters, 1.0F)) shapes.push_back(std::move(shape));
                }
            }
        }
    }
    return shapes;
}
}

RobotPhysicsAdapter::RobotPhysicsAdapter(
    Scene& scene,
    const Entity& robotRoot,
    const grasplink::robotics::models::RobotSpecification& specification,
    const ModelResource& model)
{
    if (!robotRoot) throw std::runtime_error("RobotPhysicsAdapter: invalid robot root");
    if (specification.joints == nullptr || specification.jointCount == 0)
        throw std::invalid_argument("RobotPhysicsAdapter: joint specifications are missing");
    if (specification.links == nullptr || specification.linkCount == 0)
        throw std::invalid_argument("RobotPhysicsAdapter: link specifications are missing");
    const std::vector<glm::mat4> nodeTransforms = BuildNodeWorldTransforms(model);
    std::unordered_map<std::string, const NodeData*> nodesByName;
    for (const NodeData& node : model.nodes)
        if (!nodesByName.emplace(node.name, &node).second)
            throw std::invalid_argument("RobotPhysicsAdapter: duplicate GLB node name");
    std::unordered_set<std::string> movingNodes;
    for (std::size_t i = 0; i < specification.jointCount; ++i)
        movingNodes.emplace(specification.joints[i].name);

    links_.reserve(specification.linkCount);
    for (std::size_t i = 0; i < specification.linkCount; ++i)
    {
        const auto& link = specification.links[i];
        if (link.jointIndex >= specification.jointCount)
            throw std::invalid_argument("RobotPhysicsAdapter: link joint index is out of range");
        const auto joint = nodesByName.find(std::string(specification.joints[link.jointIndex].name));
        const auto linkNode = nodesByName.find(std::string(link.name));
        if (joint == nodesByName.end() || linkNode == nodesByName.end())
            throw std::invalid_argument("RobotPhysicsAdapter: GLB joint or link node is missing");

        auto shapes = BuildLinkShapes(model, nodeTransforms, *joint->second, *linkNode->second, movingNodes);
        if (shapes.empty()) throw std::invalid_argument("RobotPhysicsAdapter: link has no GLB collision geometry");

        Entity entity = scene.CreateEntity(std::string(link.name) + "_CollisionProxy");
        // robotRoot를 부모로 두어 Base 기준 FK 자세가 Scene 배치와 함께 움직이게 한다.
        entity.SetParent(robotRoot);
        entity.Add<RobotCollisionProxy>()
            .set<RigidBody>(RigidBody{
                grasplink::physics::BodyMotionType::Kinematic,
                grasplink::physics::CollisionLayer::Robot})
            .set<Colliders>(Colliders{std::move(shapes)});
        links_.push_back({entity, link.jointIndex});
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
        // FK의 double 정밀도로 Euler를 구한 뒤 저장한다. 먼저 float로 줄이면 90°에서 오차가 커진다.
        link.entity.SetLocalRotation(glm::vec3{glm::eulerAngles(glm::normalize(glm::dquat{
            pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z}))});
    }
}
}
