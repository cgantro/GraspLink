#include "assets/GltfLoader.h"
#include "TestSupport.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{
using Bytes = std::vector<unsigned char>;

void AppendUInt32(Bytes& bytes, std::uint32_t value)
{
    for (unsigned int shift = 0; shift < 32; shift += 8)
        bytes.push_back(static_cast<unsigned char>((value >> shift) & 0xffU));
}

Bytes TriangleBytes()
{
    // 최소 POSITION fixture: 3개 FLOAT VEC3(36 bytes) 뒤에 여유 바이트를 둬 view 경계를 따로 시험한다.
    Bytes bytes(64, 0U);
    const float positions[]{0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F};
    std::memcpy(bytes.data(), positions, sizeof(positions));
    return bytes;
}

// 입력: JSON과 BIN을 분리 구성해 TinyGLTF 파싱을 포함한 실제 loader 경로를 확인한다. POSITION 값은 모델 공간 좌표다.
std::string TriangleJson(
    const std::string& accessor = R"({"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"})",
    const std::string& views = R"({"buffer":0,"byteOffset":0,"byteLength":36})",
    const std::string& nodes = R"({"mesh":0})")
{
    return R"({"asset":{"version":"2.0"},"buffers":[{"byteLength":64}],"bufferViews":[)" + views +
        R"(],"accessors":[)" + accessor + R"(],"meshes":[{"primitives":[{"attributes":{"POSITION":0}}]}],"nodes":[)" +
        nodes + R"(],"scenes":[{"nodes":[0]}],"scene":0})";
}

/**
 * @brief 임시 경로에 직접 만든 최소 GLB와 잘못된 GLB 입력을 제공하는 fixture.
 * @details JSON/BIN chunk의 4-byte 정렬과 GLB 헤더 길이를 구성한다. 각 거부 사례는 loader가 실패 이유를
 * 진단에 남기는지 확인하며, fixture 파일은 객체 수명이 끝날 때 임시 디렉터리와 함께 제거된다.
 */
class Fixtures
{
public:
    Fixtures()
    {
        directory_ = std::filesystem::temp_directory_path() /
            ("grasplink-gltf-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        std::filesystem::create_directories(directory_);
    }

    ~Fixtures()
    {
        std::error_code error;
        std::filesystem::remove_all(directory_, error);
    }

    std::filesystem::path Write(const std::string& name, std::string json, Bytes binary = TriangleBytes())
    {
        // JSON은 공백, BIN은 0으로 chunk 경계를 4 bytes에 맞춘 뒤 little-endian GLB header를 기록한다.
        while (json.size() % 4U != 0U) json.push_back(' ');
        while (binary.size() % 4U != 0U) binary.push_back(0U);
        Bytes bytes;
        AppendUInt32(bytes, 0x46546c67U);
        AppendUInt32(bytes, 2U);
        AppendUInt32(bytes, static_cast<std::uint32_t>(12U + 8U + json.size() + 8U + binary.size()));
        AppendUInt32(bytes, static_cast<std::uint32_t>(json.size()));
        AppendUInt32(bytes, 0x4e4f534aU);
        bytes.insert(bytes.end(), json.begin(), json.end());
        AppendUInt32(bytes, static_cast<std::uint32_t>(binary.size()));
        AppendUInt32(bytes, 0x004e4942U);
        bytes.insert(bytes.end(), binary.begin(), binary.end());
        const auto path = directory_ / (name + ".glb");
        std::ofstream file(path, std::ios::binary);
        file.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        Require(static_cast<bool>(file), "write GLB fixture: " + name);
        return path;
    }

    void Reject(const std::string& name, const std::string& json, const std::string& diagnostic,
        Bytes binary = TriangleBytes())
    {
        // 로더가 해당 입력을 거부하고 예상한 진단을 냈을 때만 malformed fixture로 센다.
        const auto path = Write(name, json, std::move(binary));
        try { GltfLoader::LoadGLB(path); }
        catch (const std::runtime_error& error)
        {
            Require(std::string(error.what()).find(diagnostic) != std::string::npos,
                name + " rejected for unexpected reason: " + error.what());
            ++rejected_;
            return;
        }
        throw std::runtime_error(name + " must be rejected");
    }

    int RejectedCount() const { return rejected_; }

private:
    std::filesystem::path directory_;
    int rejected_ = 0;
};

void CheckRepositoryAssets()
{
    // 저장소의 실제 로봇/바닥 GLB가 mesh와 단일 rooted node tree로 읽히는지 확인한다.
    for (const char* filename : {"HCR12A_R00.glb", "HCR12A_2F-85.glb", "plane.glb"})
    {
        const auto resource = GltfLoader::LoadGLB(std::filesystem::path(GRASPLINK_TEST_ASSET_DIR) / filename);
        Require(!resource.meshes.empty() && !resource.nodes.empty(), std::string(filename) + " has model data");
        Require(resource.rootNodeIndex >= 0 && resource.rootNodeIndex < static_cast<int>(resource.nodes.size()),
            std::string(filename) + " has a valid root");
        Require(resource.nodes[static_cast<std::size_t>(resource.rootNodeIndex)].parentIndex == -1,
            std::string(filename) + " root has no parent");
        for (const auto& mesh : resource.meshes)
        {
            Require(!mesh.vertices.empty() && !mesh.indices.empty(), std::string(filename) + " has geometry");
            for (const auto index : mesh.indices)
                Require(index < mesh.vertices.size(), std::string(filename) + " index is inside mesh");
        }
        std::cout << "Loaded " << filename << ": " << resource.meshes.size() << " meshes, "
                  << resource.nodes.size() << " nodes\n";
    }
}

void CheckAccessors(Fixtures& fixtures)
{
    // Packed/interleaved FLOAT accessor의 stride/offset 계산과 buffer 범위·overflow·sparse·0개·NaN 거부를 확인한다.
    const auto valid = GltfLoader::LoadGLB(fixtures.Write("triangle", TriangleJson()));
    Require(valid.meshes.front().vertices.size() == 3 && valid.meshes.front().indices ==
        std::vector<std::uint32_t>{0U, 1U, 2U}, "packed FLOAT positions and generated indices");
    RequireNear(valid.meshes.front().vertices[1].position.x, 1.0, 0.0, "second packed position");

    Bytes interleaved(64, 0U);
    const float positions[]{0.0F, 0.0F, 0.0F, 99.0F, 1.0F, 0.0F, 0.0F, 99.0F, 0.0F, 1.0F, 0.0F};
    std::memcpy(interleaved.data(), positions, sizeof(positions));
    const auto strided = GltfLoader::LoadGLB(fixtures.Write("interleaved", TriangleJson(
        R"({"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"})",
        R"({"buffer":0,"byteLength":44,"byteStride":16})"), interleaved));
    RequireNear(strided.meshes.front().vertices[1].position.x, 1.0, 0.0, "interleaved position skips padding");
    RequireNear(strided.meshes.front().vertices[2].position.y, 1.0, 0.0, "interleaved final position");

    const std::string sparseViews = R"({"buffer":0,"byteLength":36},{"buffer":0,"byteOffset":36,"byteLength":1},{"buffer":0,"byteOffset":40,"byteLength":12})";
    const std::string sparse = R"("componentType":5126,"count":3,"type":"VEC3","sparse":{"count":1,"indices":{"bufferView":1,"componentType":5121},"values":{"bufferView":2}})";
    fixtures.Reject("sparse-with-base", TriangleJson("{\"bufferView\":0," + sparse + "}", sparseViews), "Sparse glTF accessors");
    fixtures.Reject("sparse-without-base", TriangleJson("{" + sparse + "}", sparseViews), "Sparse glTF accessors");
    fixtures.Reject("view-outside-buffer", TriangleJson(
        R"({"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"})",
        R"({"buffer":0,"byteOffset":60,"byteLength":36})"), "bufferView exceeds buffer size");
    fixtures.Reject("view-offset-overflow", TriangleJson(
        R"({"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"})",
        R"({"buffer":0,"byteOffset":18446744073709551612,"byteLength":36})"), "bufferView exceeds buffer size");
    fixtures.Reject("accessor-crosses-view", TriangleJson(
        R"({"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"})",
        R"({"buffer":0,"byteLength":24})"), "accessor exceeds bufferView");
    fixtures.Reject("accessor-offset-outside-view", TriangleJson(
        R"({"bufferView":0,"byteOffset":40,"componentType":5126,"count":3,"type":"VEC3"})"), "accessor offset exceeds bufferView");
    fixtures.Reject("accessor-range-overflow", TriangleJson(
        R"({"bufferView":0,"componentType":5126,"count":18446744073709551615,"type":"VEC3"})"), "accessor range overflows");
    fixtures.Reject("stride-smaller-than-element", TriangleJson(
        R"({"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"})",
        R"({"buffer":0,"byteLength":36,"byteStride":8})"), "Invalid glTF accessor stride");
    // count가 0이면 buffer 끝 주소에서 읽지 않고 빈 mesh로 거부한다.
    fixtures.Reject("zero-count-at-buffer-end", TriangleJson(
        R"({"bufferView":0,"componentType":5126,"count":0,"type":"VEC3"})",
        R"({"buffer":0,"byteOffset":64,"byteLength":0})"), "zero vertices");
    auto nonfinite = TriangleBytes();
    const float nan = std::numeric_limits<float>::quiet_NaN();
    std::memcpy(nonfinite.data(), &nan, sizeof(nan));
    fixtures.Reject("nonfinite-position", TriangleJson(), "non-finite value", nonfinite);
}

void CheckTransformsAndHierarchy(Fixtures& fixtures)
{
    // 노드 matrix는 비퇴화 TRS로 분해되고, 잘못된 원근/기울기/스케일/quaternion과 순환·다중 부모 입력은 거부돼야 한다.
    const std::string accessor = R"({"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"})";
    const std::string view = R"({"buffer":0,"byteLength":36})";
    const auto translated = GltfLoader::LoadGLB(fixtures.Write("valid-matrix", TriangleJson(accessor, view,
        R"({"mesh":0,"matrix":[2,0,0,0,0,3,0,0,0,0,4,0,5,6,7,1]})")));
    RequireNear(translated.nodes.front().translation.x, 5.0, 1e-5, "matrix local translation");
    RequireNear(translated.nodes.front().scale.z, 4.0, 1e-5, "matrix local scale");
    fixtures.Reject("matrix-skew", TriangleJson(accessor, view,
        R"({"mesh":0,"matrix":[1,0,0,0,0.25,1,0,0,0,0,1,0,0,0,0,1]})"), "non-degenerate TRS only");
    fixtures.Reject("matrix-perspective", TriangleJson(accessor, view,
        R"({"mesh":0,"matrix":[1,0,0,0.25,0,1,0,0,0,0,1,0,0,0,0,1]})"), "Perspective node matrices");
    fixtures.Reject("matrix-degenerate", TriangleJson(accessor, view,
        R"({"mesh":0,"matrix":[0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]})"), "decompose glTF node matrix");
    fixtures.Reject("matrix-float-overflow", TriangleJson(accessor, view,
        R"({"mesh":0,"matrix":[1e300,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]})"), "non-finite value");
    fixtures.Reject("trs-zero-scale", TriangleJson(accessor, view,
        R"({"mesh":0,"scale":[1,0,1]})"), "scale must be non-degenerate");
    fixtures.Reject("trs-zero-quaternion", TriangleJson(accessor, view,
        R"({"mesh":0,"rotation":[0,0,0,0]})"), "rotation quaternion is zero");
    fixtures.Reject("trs-translation-overflow", TriangleJson(accessor, view,
        R"({"mesh":0,"translation":[1e300,0,0]})"), "non-finite value");
    fixtures.Reject("trs-scale-overflow", TriangleJson(accessor, view,
        R"({"mesh":0,"scale":[1e300,1,1]})"), "non-finite value");
    fixtures.Reject("trs-rotation-overflow", TriangleJson(accessor, view,
        R"({"mesh":0,"rotation":[1e300,0,0,1]})"), "non-finite value");
    fixtures.Reject("node-self-cycle", TriangleJson(accessor, view,
        R"({"mesh":0,"children":[0]})"), "hierarchy contains a cycle");
    fixtures.Reject("node-two-cycle", TriangleJson(accessor, view,
        R"({"children":[1]},{"mesh":0,"children":[0]})"), "hierarchy contains a cycle");
    fixtures.Reject("node-multiple-parents", TriangleJson(accessor, view,
        R"({"children":[2]},{"children":[2]},{"mesh":0})"), "multiple parents");
    fixtures.Reject("node-disconnected-from-scene", TriangleJson(accessor, view,
        R"({"mesh":0},{"mesh":0})"), "single tree under the scene root");
}

/**
 * @brief glTF의 xyzw 순서로 Local TRS quaternion fixture를 만든다.
 * @details GLM 생성자 순서인 wxyz와 구분해 기록한다. float 정밀도를 보존하므로 JSON 출력 오차가
 * pitch 90도 부근의 회전 보존 검사를 가리지 않는다.
 */
std::string QuaternionTrsNode(const glm::vec3& position, const glm::quat& rotation, const glm::vec3& scale)
{
    std::ostringstream json;
    json << std::setprecision(std::numeric_limits<float>::max_digits10)
         << "{\"mesh\":0,\"translation\":[" << position.x << ',' << position.y << ',' << position.z
         << "],\"rotation\":[" << rotation.x << ',' << rotation.y << ',' << rotation.z << ',' << rotation.w
         << "],\"scale\":[" << scale.x << ',' << scale.y << ',' << scale.z << "]}";
    return json.str();
}

/** @brief GLM 열 벡터 행렬을 glTF의 column-major matrix 배열로 기록한다. */
std::string MatrixNode(const glm::mat4& matrix)
{
    std::ostringstream json;
    json << std::setprecision(std::numeric_limits<float>::max_digits10) << "{\"mesh\":0,\"matrix\":[";
    for (int column = 0; column < 4; ++column)
        for (int row = 0; row < 4; ++row)
        {
            if (column != 0 || row != 0) json << ',';
            json << matrix[column][row];
        }
    json << "]}";
    return json.str();
}

void RequireNodeQuaternion(const NodeData& node, const glm::quat& expected, const std::string& label)
{
    RequireNear(glm::length(node.rotation), 1.0, 1e-5, label + " unit quaternion");
    // quaternion은 부호가 반대여도 같은 방향이다. matrix 분해가 선택하는 부호에 의존하지 않는다.
    RequireNear(std::abs(glm::dot(node.rotation, glm::normalize(expected))), 1.0, 1e-5,
        label + " quaternion direction");
}

void RequireNodeMatrix(const NodeData& node, const glm::mat4& expected, const std::string& label)
{
    const glm::mat4 actual = glm::translate(glm::mat4(1.0F), node.translation) *
        glm::mat4_cast(node.rotation) * glm::scale(glm::mat4(1.0F), node.scale);
    for (int column = 0; column < 4; ++column)
        for (int row = 0; row < 4; ++row)
            RequireNear(actual[column][row], expected[column][row], 1e-5,
                label + " [" + std::to_string(column) + "][" + std::to_string(row) + "]");
}

/**
 * @brief GLB TRS와 matrix 입력이 quaternion 방향과 Local 행렬을 보존하는지 확인한다.
 * @details 기대 회전은 축 회전 quaternion을 직접 곱해 만든다. Euler 각으로 되돌리는 과정 없이
 * pitch 90도 양쪽의 복합 회전, glTF xyzw 순서, 반대 부호와 비단위 입력을 비교한다.
 */
void CheckQuaternionTransforms(Fixtures& fixtures)
{
    const std::string accessor = R"({"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"})";
    const std::string view = R"({"buffer":0,"byteLength":36})";
    const glm::vec3 position{0.8F, -1.2F, 0.4F};
    const glm::vec3 scale{1.5F, 0.7F, 2.0F};
    const glm::quat ordered = glm::normalize(glm::quat{0.9F, 0.1F, -0.2F, 0.3F});
    const auto orderedResource = GltfLoader::LoadGLB(fixtures.Write("quaternion-xyzw",
        TriangleJson(accessor, view, QuaternionTrsNode(position, ordered, scale))));
    const glm::quat actualOrdered = orderedResource.nodes.front().rotation;
    RequireNear(actualOrdered.w, ordered.w, 1e-6, "glTF xyzw maps w to quaternion scalar");
    RequireNear(actualOrdered.x, ordered.x, 1e-6, "glTF xyzw maps x");
    RequireNear(actualOrdered.y, ordered.y, 1e-6, "glTF xyzw maps y");
    RequireNear(actualOrdered.z, ordered.z, 1e-6, "glTF xyzw maps z");

    const auto defaults = GltfLoader::LoadGLB(fixtures.Write("quaternion-default", TriangleJson()));
    RequireNodeQuaternion(defaults.nodes.front(), glm::quat{1.0F, 0.0F, 0.0F, 0.0F}, "default rotation");

    const float halfPi = std::acos(-1.0F) * 0.5F;
    int fixtureIndex = 0;
    for (const float pitch : {halfPi - 1.0e-5F, halfPi, halfPi + 1.0e-5F})
    {
        const glm::quat expectedRotation = glm::normalize(
            glm::angleAxis(-0.8F, glm::vec3{0.0F, 0.0F, 1.0F}) *
            glm::angleAxis(pitch, glm::vec3{0.0F, 1.0F, 0.0F}) *
            glm::angleAxis(0.4F, glm::vec3{1.0F, 0.0F, 0.0F}));
        const glm::mat4 expectedMatrix = glm::translate(glm::mat4(1.0F), position) *
            glm::mat4_cast(expectedRotation) * glm::scale(glm::mat4(1.0F), scale);
        for (const float multiplier : {1.0F, -1.0F, 7.0F})
        {
            const std::string name = "quaternion-trs-" + std::to_string(fixtureIndex++);
            const auto resource = GltfLoader::LoadGLB(fixtures.Write(name, TriangleJson(accessor, view,
                QuaternionTrsNode(position, expectedRotation * multiplier, scale))));
            RequireNodeQuaternion(resource.nodes.front(), expectedRotation, name);
            RequireNodeMatrix(resource.nodes.front(), expectedMatrix, name + " Local TRS");
        }
        const std::string name = "quaternion-matrix-" + std::to_string(fixtureIndex++);
        const auto resource = GltfLoader::LoadGLB(fixtures.Write(name,
            TriangleJson(accessor, view, MatrixNode(expectedMatrix))));
        RequireNodeQuaternion(resource.nodes.front(), expectedRotation, name);
        RequireNodeMatrix(resource.nodes.front(), expectedMatrix, name + " decomposed Local TRS");
    }
}
}

/**
 * @brief 실제 저장소 모델과 직접 만든 malformed GLB fixture에서 loader 계약을 확인한다.
 * @details 입력은 임시 파일로 구성해 accessor 메모리 범위와 노드 계층 검증을 회귀 검사한다. TRS와 matrix의
 * quaternion 방향 및 특이 자세 부근의 Local 행렬을 비교한다. Mesh vertex 위치는 원래 GLB 좌표값을
 * 보존하며, 이 테스트 자체는 OpenGL 업로드나 렌더링을 수행하지 않는다.
 */
int main()
{
    try
    {
        CheckRepositoryAssets();
        Fixtures fixtures;
        CheckAccessors(fixtures);
        CheckTransformsAndHierarchy(fixtures);
        CheckQuaternionTransforms(fixtures);
        std::cout << "GLB packed/interleaved/quaternion/matrix checks and " << fixtures.RejectedCount()
                  << " malformed fixtures passed\n";
        return 0;
    }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
