#include "assets/GltfLoader.h"
#include "TestSupport.h"

#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
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
    Bytes bytes(64, 0U);
    const float positions[]{0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F};
    std::memcpy(bytes.data(), positions, sizeof(positions));
    return bytes;
}

// 입력: JSON과 BIN을 별도로 구성해 TinyGLTF 파싱부터 실제 loader 경로를 확인한다.
std::string TriangleJson(
    const std::string& accessor = R"({"bufferView":0,"componentType":5126,"count":3,"type":"VEC3"})",
    const std::string& views = R"({"buffer":0,"byteOffset":0,"byteLength":36})",
    const std::string& nodes = R"({"mesh":0})")
{
    return R"({"asset":{"version":"2.0"},"buffers":[{"byteLength":64}],"bufferViews":[)" + views +
        R"(],"accessors":[)" + accessor + R"(],"meshes":[{"primitives":[{"attributes":{"POSITION":0}}]}],"nodes":[)" +
        nodes + R"(],"scenes":[{"nodes":[0]}],"scene":0})";
}

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
}

int main()
{
    try
    {
        CheckRepositoryAssets();
        Fixtures fixtures;
        CheckAccessors(fixtures);
        CheckTransformsAndHierarchy(fixtures);
        std::cout << "GLB packed/interleaved/matrix checks and " << fixtures.RejectedCount()
                  << " malformed fixtures passed\n";
        return 0;
    }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
