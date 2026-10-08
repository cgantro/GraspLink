#include "simulation/robotics/RobotCollisionGeometryBuilder.h"
#include "CollisionGeometry.h"

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

namespace grasplink::simulation::robot_collision_geometry
{
using grasplink::model::MeshData;
using grasplink::model::ModelResource;
using grasplink::model::NodeData;
using grasplink::model::SubMeshInfo;

namespace
{
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

PositionKey MakePositionKey(const glm::vec3& position, const Options& options)
{
    // 정점 좌표를 약 0.01 mm 간격의 격자에 맞춰 비교한다. 그래서 서로 다른 index를 가진 seam(메시 이음새)의 같은 위치 정점을 하나로 인식한다.
    const double precision = std::round(1.0 / options.vertexMergeToleranceMeters);
    return {
        static_cast<std::int64_t>(std::llround(position.x * precision)),
        static_cast<std::int64_t>(std::llround(position.y * precision)),
        static_cast<std::int64_t>(std::llround(position.z * precision))};
}

PositionKey MakeCollisionCellKey(const glm::vec3& position, const Options& options)
{
    // 단위: 충돌 근사 셀 한 변은 16 cm다. 긴 링크의 작은 충돌 형상 수를 줄이면서 떨어진 부품 사이 빈 공간은 한 덩어리로 합치지 않는다.
    return {
        static_cast<std::int64_t>(std::floor(position.x / options.cellSizeMeters)),
        static_cast<std::int64_t>(std::floor(position.y / options.cellSizeMeters)),
        static_cast<std::int64_t>(std::floor(position.z / options.cellSizeMeters))};
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
    std::size_t jointIndex,
    std::size_t linkIndex,
    const std::unordered_set<std::string>& movingNodes,
    const Options& options)
{
    using Shape = grasplink::physics::CollisionShapeDescription;
    std::vector<Shape> shapes;
    if (jointIndex >= model.nodes.size() || linkIndex >= model.nodes.size() ||
        nodeTransforms.size() != model.nodes.size())
        throw std::invalid_argument("Robot collision geometry: node transform index is out of range");
    // 메시 정점을 GLB 좌표에서 이 Link를 움직이는 joint 원점 좌표로 바꾼다. 실행 중 FK로 계산한 관절 자세는 별도 proxy Entity에 적용한다.
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
            // 다음 가동 관절 아래의 메시부터는 다른 Link가 움직이므로 이 Link 형상에 포함하지 않는다. Gripper 메시도 팔 어댑터 범위에서 제외한다.
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
                    MakePositionKey(mesh.vertices[vertexIndex].position, options), vertexIndex);
                if (!inserted) Join(parents, vertexIndex, iterator->second);
            }

            // 좌표가 같은 seam 정점과 삼각형으로 연결된 정점을 묶어 연결 component를 만든다. 이 묶음으로 서로 떨어진 메시 부품을 구분한다.
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
            // 연결된 각 부품 안에서 삼각형 중심이 들어가는 16 cm 셀에 삼각형 정점을 모은다. 삼각형 자체는 셀 경계에서 잘라 나누지 않는다.
            for (std::size_t index = subMesh.indexStart; index + 2 < end; index += 3)
            {
                const std::uint32_t first = mesh.indices[index];
                const std::size_t component = FindRoot(parents, first);
                const glm::vec3 a = glm::vec3(meshToJoint * glm::vec4(mesh.vertices[first].position, 1.0F));
                const glm::vec3 b = glm::vec3(meshToJoint * glm::vec4(mesh.vertices[mesh.indices[index + 1]].position, 1.0F));
                const glm::vec3 c = glm::vec3(meshToJoint * glm::vec4(mesh.vertices[mesh.indices[index + 2]].position, 1.0F));
                const PositionKey cell = MakeCollisionCellKey((a + b + c) / 3.0F, options);
                auto& points = collisionCells[component][cell];
                points.insert(points.end(), {a, b, c});
            }

            for (const auto& [component, cells] : collisionCells)
            {
                // 연결 부품 전체 폭이 4 cm보다 작거나 셀 안 정점 폭이 1 cm보다 작으면 충돌 형상에서 제외한다.
                // 시각 Mesh와 단순화한 충돌 외피의 크기와 윤곽은 완전히 같지 않을 수 있다.
                if (!HasExtent(components[component], options.componentMinimumExtentMeters)) continue;
                for (const auto& [cell, vertices] : cells)
                {
                    (void)cell;
                    if (!HasExtent(vertices, options.cellMinimumExtentMeters)) continue;
                    std::vector<glm::vec3> uniqueVertices;
                    std::unordered_set<PositionKey, PositionKeyHash> uniquePositions;
                    uniqueVertices.reserve(vertices.size());
                    uniquePositions.reserve(vertices.size());
                    for (const glm::vec3& vertex : vertices)
                    {
                        if (uniquePositions.insert(MakePositionKey(vertex, options)).second)
                            uniqueVertices.push_back(vertex);
                    }
                    Shape shape;
                    shape.type = grasplink::physics::CollisionShapeType::ConvexHull;
                    shape.pointsMeters = detail::BuildConvexSupportPoints(uniqueVertices);
                    // 여러 방향에서 가장 바깥에 있는 정점으로 만든 근사 볼록 외피를 검사한다. 1 m 기준으로 실제 부피가 있다고 판단된 셀만 충돌 형상으로 남긴다.
                    if (detail::HasHullVolume(shape.pointsMeters, 1.0F)) shapes.push_back(std::move(shape));
                }
            }
        }
    }
    return shapes;
}

std::vector<grasplink::physics::CollisionShapeDescription> BuildBaseShapes(
    const ModelResource& model,
    const std::vector<glm::mat4>& nodeTransforms,
    const std::unordered_map<std::string, std::size_t>& nodesByName,
    const std::unordered_set<std::string>& movingNodes,
    const Options& options)
{
    const auto baseIterator = nodesByName.find("Base");
    if (baseIterator == nodesByName.end()) return {};

    const std::size_t baseIndex = baseIterator->second;
    // Base 메시도 Link와 같은 연결 부품 및 16 cm 셀 기준으로 나눠 실제 오목한 공간을 볼록 껍질 하나로 메우지 않는다.
    auto shapes = BuildLinkShapes(model, nodeTransforms, baseIndex, baseIndex, movingNodes, options);
    // 분할된 Base 기준 정점을 proxy 자식 Entity가 사용할 RobotRoot 기준 좌표로 옮긴다.
    if (model.rootNodeIndex >= 0 &&
        static_cast<std::size_t>(model.rootNodeIndex) >= nodeTransforms.size())
        throw std::invalid_argument("Robot collision geometry: GLB root node is out of range");
    const glm::mat4 rootInverse = model.rootNodeIndex < 0
        ? glm::mat4(1.0F)
        : glm::inverse(nodeTransforms[static_cast<std::size_t>(model.rootNodeIndex)]);
    const glm::mat4 baseToRobotRoot = rootInverse * nodeTransforms[baseIndex];
    for (auto& shape : shapes)
        for (glm::vec3& point : shape.pointsMeters)
            point = glm::vec3(baseToRobotRoot * glm::vec4(point, 1.0F));
    return shapes;
}
}

Result Build(
    const grasplink::robotics::models::RobotSpecification& specification,
    const ModelResource& model,
    const Options& options)
{
    if (specification.joints == nullptr || specification.jointCount == 0)
        throw std::invalid_argument("Robot collision geometry: joint specifications are missing");
    if (specification.links == nullptr || specification.linkCount == 0)
        throw std::invalid_argument("Robot collision geometry: link specifications are missing");
    if (!std::isfinite(options.vertexMergeToleranceMeters) ||
        !std::isfinite(options.cellSizeMeters) ||
        !std::isfinite(options.componentMinimumExtentMeters) ||
        !std::isfinite(options.cellMinimumExtentMeters) ||
        !(options.vertexMergeToleranceMeters > 0.0) || !(options.cellSizeMeters > 0.0F) ||
        !(options.componentMinimumExtentMeters >= 0.0F) || !(options.cellMinimumExtentMeters >= 0.0F))
        throw std::invalid_argument("Robot collision geometry: geometry options are invalid");

    const std::vector<glm::mat4> nodeTransforms = detail::BuildNodeWorldTransforms(model);
    std::unordered_map<std::string, std::size_t> nodesByName;
    for (std::size_t index = 0; index < model.nodes.size(); ++index)
        if (!nodesByName.emplace(model.nodes[index].name, index).second)
            throw std::invalid_argument("Robot collision geometry: duplicate GLB node name");

    std::unordered_set<std::string> movingNodes;
    for (std::size_t i = 0; i < specification.jointCount; ++i)
        movingNodes.emplace(specification.joints[i].name);

    Result result;
    result.baseShapes = BuildBaseShapes(model, nodeTransforms, nodesByName, movingNodes, options);
    result.links.reserve(specification.linkCount);
    for (std::size_t i = 0; i < specification.linkCount; ++i)
    {
        const auto& link = specification.links[i];
        if (link.jointIndex >= specification.jointCount)
            throw std::invalid_argument("Robot collision geometry: link joint index is out of range");
        const auto joint = nodesByName.find(std::string(specification.joints[link.jointIndex].name));
        const auto linkNode = nodesByName.find(std::string(link.name));
        if (joint == nodesByName.end() || linkNode == nodesByName.end())
            throw std::invalid_argument("Robot collision geometry: GLB joint or link node is missing");

        auto shapes = BuildLinkShapes(model, nodeTransforms, joint->second, linkNode->second,
            movingNodes, options);
        if (shapes.empty())
            throw std::invalid_argument("Robot collision geometry: link has no GLB collision geometry");
        result.links.push_back({link.jointIndex, std::move(shapes)});
    }
    return result;
}
}
