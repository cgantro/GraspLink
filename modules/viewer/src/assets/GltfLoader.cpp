#include "assets/GltfLoader.h"

#include <tiny_gltf.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/quaternion.hpp>

#include <cstdint>
#include <cmath>
#include <cstring>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
// Buffer는 glTF 파일의 원본 byte 묶음이고 BufferView는 그중 일부 구간이다. Accessor는 그 구간에서 읽을 값의 자료형·개수·간격을 적은 기록이다.
// 세 기록의 연결과 범위를 확인한 뒤 첫 값의 주소와 다음 값까지의 간격을 돌려준다.
struct AccessorView
{
    const unsigned char* data = nullptr;
    std::size_t stride = 0;
    std::size_t count = 0;
};

// Accessor가 가리키는 byte가 실제 Buffer 범위 안에 있는지 확인해 읽을 주소를 만든다. 기존 값 일부를 덮어쓰는 Sparse 형식은 현재 지원하지 않는다.
AccessorView GetAccessorView(
    const tinygltf::Model& model,
    int accessorIndex,
    std::size_t elementSize)
{
    if (accessorIndex < 0 || accessorIndex >= static_cast<int>(model.accessors.size()))
        throw std::runtime_error("Invalid glTF accessor Index");

    const tinygltf::Accessor& accessor = model.accessors[accessorIndex];

    if (accessor.sparse.isSparse)
        throw std::runtime_error("Sparse glTF accessors are not supported");

    if (accessor.bufferView < 0 ||
        accessor.bufferView >= static_cast<int>(model.bufferViews.size()))
    {
        throw std::runtime_error("Accessor without BufferView is not supported");
    }

    const tinygltf::BufferView& bufferView = model.bufferViews[accessor.bufferView];
    if (bufferView.buffer < 0 ||
        bufferView.buffer >= static_cast<int>(model.buffers.size()))
    {
        throw std::runtime_error("Invalid glTF buffer index");
    }

    const tinygltf::Buffer& buffer = model.buffers[bufferView.buffer];

    /*
        byteStride는 한 값의 첫 byte에서 다음 같은 값의 첫 byte까지 간격이다.
        예를 들어 정점마다 [위치][법선][UV]를 붙여 저장하면 다음 위치는 위치 값만큼이 아니라 법선과 UV까지 건너뛴 뒤 시작한다.
        마지막 값의 끝까지 BufferView 안에 드는지 확인하므로 잘못된 파일이 범위 밖 주소를 만들기 전에 거부된다.
    */
    const int byteStride = accessor.ByteStride(bufferView);
    if (byteStride <= 0 || static_cast<std::size_t>(byteStride) < elementSize)
        throw std::runtime_error("Invalid glTF accessor stride");

    const std::size_t bufferSize = buffer.data.size();
    const std::size_t viewOffset = static_cast<std::size_t>(bufferView.byteOffset);
    const std::size_t viewLength = static_cast<std::size_t>(bufferView.byteLength);
    if (viewOffset > bufferSize || viewLength > bufferSize - viewOffset)
        throw std::runtime_error("glTF bufferView exceeds buffer size");

    const std::size_t accessorOffset = static_cast<std::size_t>(accessor.byteOffset);
    if (accessorOffset > viewLength)
        throw std::runtime_error("glTF accessor offset exceeds bufferView");
    if (accessorOffset > std::numeric_limits<std::size_t>::max() - viewOffset)
        throw std::runtime_error("glTF accessor offset overflows");
    const std::size_t offset = viewOffset + accessorOffset;

    if (accessor.count > 0)
    {
        const std::size_t count = static_cast<std::size_t>(accessor.count);
        const std::size_t stride = static_cast<std::size_t>(byteStride);
        if (accessorOffset > std::numeric_limits<std::size_t>::max() - elementSize ||
            count - 1U > (std::numeric_limits<std::size_t>::max() - accessorOffset - elementSize) / stride)
            throw std::runtime_error("glTF accessor range overflows");
        const std::size_t accessorEnd = accessorOffset + (count - 1U) * stride + elementSize;
        if (accessorEnd > viewLength)
            throw std::runtime_error("glTF accessor exceeds bufferView");
    }

    return AccessorView{
        accessor.count == 0 ? nullptr : buffer.data.data() + offset,
        static_cast<std::size_t>(byteStride),
        static_cast<std::size_t>(accessor.count)};
}

void RequireFinite(const glm::vec2& value)
{
    if (!std::isfinite(value.x) || !std::isfinite(value.y))
        throw std::runtime_error("glTF vertex attribute contains a non-finite value");
}

void RequireFinite(const glm::vec3& value)
{
    if (!std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.z))
        throw std::runtime_error("glTF vertex or transform contains a non-finite value");
}

void RequireFinite(const glm::vec4& value)
{
    if (!std::isfinite(value.x) || !std::isfinite(value.y) ||
        !std::isfinite(value.z) || !std::isfinite(value.w))
    {
        throw std::runtime_error("glTF transform contains a non-finite value");
    }
}

void RequireFinite(const glm::quat& value)
{
    if (!std::isfinite(value.x) || !std::isfinite(value.y) ||
        !std::isfinite(value.z) || !std::isfinite(value.w))
    {
        throw std::runtime_error("glTF rotation contains a non-finite value");
    }
}

// 정점 위치와 법선은 세 실수로 읽는다. 현재 렌더러는 접선 속성을 사용하지 않는다.
std::vector<glm::vec3> ReadVec3FloatAccessor(
    const tinygltf::Model& model,
    int accessorIndex)
{
    const tinygltf::Accessor& accessor = model.accessors.at(accessorIndex);
    if (accessor.type != TINYGLTF_TYPE_VEC3 ||
        accessor.componentType != TINYGLTF_COMPONENT_TYPE_FLOAT)
        throw std::runtime_error("Expected FLOAT VEC3 ACCESSOR");

    const AccessorView view = GetAccessorView(model, accessorIndex, sizeof(float) * 3U);

    std::vector<glm::vec3> result(view.count);

    for (std::size_t i = 0; i < view.count; ++i)
    {
        const unsigned char* source = view.data + i * view.stride;
        float values[4]{};

        // 파일의 값 시작 주소가 C++ float 정렬에 맞는다고 가정하지 않고 byte를 복사한다.
        std::memcpy(values, source, sizeof(float) * 3U);
        result[i] = glm::vec3{values[0], values[1], values[2]};
        RequireFinite(result[i]);
    }

    return result;
}

// UV는 표면에서 이미지의 어느 위치를 읽을지 나타내는 좌표다. 이 함수는 각 정점의 2개 실수 UV를 읽는다.
std::vector<glm::vec2> ReadVec2FloatAccessor(
    const tinygltf::Model& model,
    int accessorIndex)
{
    const tinygltf::Accessor& accessor = model.accessors.at(accessorIndex);
    if (accessor.type != TINYGLTF_TYPE_VEC2 ||
        accessor.componentType != TINYGLTF_COMPONENT_TYPE_FLOAT)
    {
        throw std::runtime_error("Expected FLOAT VEC2 ACCESSOR");
    }

    constexpr std::size_t kElementSize = sizeof(float) * 2U;
    const AccessorView view = GetAccessorView(model, accessorIndex, kElementSize);

    std::vector<glm::vec2> result(view.count);

    for (std::size_t i = 0; i < view.count; ++i)
    {
        const unsigned char* source = view.data + i * view.stride;
        float values[2]{};
        std::memcpy(values, source, sizeof(values));
        result[i] = glm::vec2{values[0], values[1]};
        RequireFinite(result[i]);
    }

    return result;
}

// 파일에서 읽을 값 주소가 C++ 타입 T의 정렬 경계에 맞는다고 가정하지 않고 복사한다.
// glTF 파일이 요구하는 별도의 정렬 조건은 여기서 검사하지 않는다.
template<typename T>
T ReadScalar(const unsigned char* data)
{
    T value{};
    std::memcpy(&value, data, sizeof(T));
    return value;
}

// 삼각형 연결 번호는 정점 배열에서 사용할 꼭짓점을 고르는 값이다. 이를 32-bit 정수로 통일하고 파일에 번호가 없으면 0부터 정점 순서대로 만든다.
std::vector<std::uint32_t> ReadIndices(
    const tinygltf::Model& model,
    int accessorIndex,
    std::size_t vertexCount)
{
    if (accessorIndex < 0)
    {
        std::vector<std::uint32_t> indices(vertexCount);
        std::iota(indices.begin(), indices.end(), 0U);
        return indices;
    }

    const tinygltf::Accessor& accessor = model.accessors.at(accessorIndex);
    if (accessor.type != TINYGLTF_TYPE_SCALAR)
        throw std::runtime_error("Index Accessor must be SCALAR");

    if (accessor.componentType != TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE &&
        accessor.componentType != TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT &&
        accessor.componentType != TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT)
    {
        throw std::runtime_error("Unsupported glTF index component type");
    }
    const std::size_t componentSize = accessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE
        ? sizeof(std::uint8_t)
        : accessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT
            ? sizeof(std::uint16_t)
            : sizeof(std::uint32_t);

    const AccessorView view = GetAccessorView(model, accessorIndex, componentSize);
    std::vector<std::uint32_t> result;
    result.reserve(view.count);

    for (std::size_t i = 0; i < view.count; ++i)
    {
        const unsigned char* source = view.data + i * view.stride;
        const std::uint32_t value = accessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE
            ? ReadScalar<std::uint8_t>(source)
            : accessor.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT
                ? ReadScalar<std::uint16_t>(source)
                : ReadScalar<std::uint32_t>(source);

        if (value >= vertexCount)
            throw std::runtime_error("glTF index exceeds vertex Count");

        result.push_back(value);
    }

    return result;
}

// 디코드된 이미지의 RGB/RGBA 채널은 각 8 bit다. Texture 번호는 이미지 번호가 아니라 glTF 재질에서 참조할 glTF texture 항목 번호다.
TextureData ConvertTexture(
    const tinygltf::Model& model,
    const tinygltf::Texture& source,
    const std::string& resourcePrefix,
    std::size_t textureIndex)
{
    TextureData result;
    result.uniqueID = ResourceID{
        resourcePrefix + "#texture/" + std::to_string(textureIndex)};

    if (source.source < 0 ||
        source.source >= static_cast<int>(model.images.size()))
    {
        throw std::runtime_error("Invalid glTF texture source");
    }

    const tinygltf::Image& image = model.images[source.source];

    if (image.width <= 0 || image.height <= 0)
        throw std::runtime_error("Invalid glTF image size");

    if (image.component != 3 && image.component != 4)
        throw std::runtime_error("Only RGB/RGBA glTF images are supported");

    if (image.bits != 8)
        throw std::runtime_error("Only 8-bit glTF images are supported");

    if (image.image.empty())
        throw std::runtime_error("glTF image contains no decoded pixels");

    result.width = image.width;
    result.height = image.height;
    result.channels = image.component;
    result.pixels = image.image;

    return result;
}

// 재질은 표면 색과 빛 반응을 정하는 데이터다. 파일의 기본색·금속성·거칠기·발광 계수와 기본색 이미지 참조를 내부 자료로 옮긴다.
MaterialData ConvertMaterial(
    const tinygltf::Material& source,
    const std::string& resourcePrefix,
    std::size_t materialIndex)
{
    MaterialData result;
    result.name = source.name;
    result.uniqueID = ResourceID{
        resourcePrefix + "#material/" + std::to_string(materialIndex)};

    const auto& pbr = source.pbrMetallicRoughness;

    if (pbr.baseColorFactor.size() == 4)
    {
        result.baseColorFactor = glm::vec4{
            static_cast<float>(pbr.baseColorFactor[0]),
            static_cast<float>(pbr.baseColorFactor[1]),
            static_cast<float>(pbr.baseColorFactor[2]),
            static_cast<float>(pbr.baseColorFactor[3])};
    }

    result.metallicFactor = static_cast<float>(pbr.metallicFactor);
    result.roughnessFactor = static_cast<float>(pbr.roughnessFactor);

    // glTF 재질은 texture 항목 번호를 가리키지만 내부 캐시는 모델 경로와 자원 종류, 해당 texture 항목 번호를 합친 ID로 찾는다.
    if (pbr.baseColorTexture.index >= 0)
    {
        result.baseColorTexture = ResourceID{
            resourcePrefix + "#texture/" +
            std::to_string(pbr.baseColorTexture.index)};
    }

    return result;
}

// Primitive는 재질 하나로 그리는 삼각형 묶음이다. 각 묶음에서 0부터 시작하는 연결 번호에 앞서 추가한 정점 수를 더해 하나의 Mesh 배열로 합친다.
MeshData ConvertMesh(
    const tinygltf::Model& model,
    const tinygltf::Mesh& source,
    const std::string& resourcePrefix,
    std::size_t meshIndex)
{
    MeshData result;
    result.name = source.name;
    result.uniqueID = ResourceID{
        resourcePrefix + "#mesh/" + std::to_string(meshIndex)};

    for (const tinygltf::Primitive& primitive : source.primitives)
    {
        if (primitive.mode != TINYGLTF_MODE_TRIANGLES && primitive.mode != -1)
            throw std::runtime_error("Only TRIANGLES glTF primitives are supported");

        // POSITION은 표면 삼각형의 각 꼭짓점 위치다. 위치가 없으면 화면에 표면을 놓을 수 없다.
        const auto positionIterator = primitive.attributes.find("POSITION");
        if (positionIterator == primitive.attributes.end())
            throw std::runtime_error("glTF primitive has no POSITION");

        const std::vector<glm::vec3> positions =
            ReadVec3FloatAccessor(model, positionIterator->second);

        const std::size_t vertexCount = positions.size();
        if (vertexCount == 0)
            throw std::runtime_error("glTF Primitives has zero vertices");
        constexpr std::size_t kMaxMeshValue = std::numeric_limits<std::uint32_t>::max();
        if (vertexCount > kMaxMeshValue || result.vertices.size() > kMaxMeshValue - vertexCount)
            throw std::runtime_error("glTF mesh vertex range exceeds uint32_t");

        // NORMAL은 표면 방향, TEXCOORD_0은 기본 색 이미지 좌표다. 선택 속성이 없으면 Vertex 기본값을 유지한다.
        std::vector<glm::vec3> normals;
        const auto normalIterator = primitive.attributes.find("NORMAL");
        if (normalIterator != primitive.attributes.end())
        {
            normals = ReadVec3FloatAccessor(model, normalIterator->second);
            if (normals.size() != vertexCount)
                throw std::runtime_error("NORMAL count does not match POSITION count");
        }

        std::vector<glm::vec2> texCoords;
        const auto texCoordIterator = primitive.attributes.find("TEXCOORD_0");
        if (texCoordIterator != primitive.attributes.end())
        {
            texCoords = ReadVec2FloatAccessor(model, texCoordIterator->second);
            if (texCoords.size() != vertexCount)
                throw std::runtime_error("TEXCOORD_0 count does not match POSITION count");
        }

        const std::uint32_t baseVertex =
            static_cast<std::uint32_t>(result.vertices.size());

        result.vertices.reserve(result.vertices.size() + vertexCount);

        for (std::size_t i = 0; i < vertexCount; ++i)
        {
            Vertex vertex;
            vertex.position = positions[i];
            if (!normals.empty()) vertex.normal = normals[i];
            if (!texCoords.empty()) vertex.texCoord = texCoords[i];
            result.vertices.push_back(vertex);
        }

        const std::vector<std::uint32_t> localIndices =
            ReadIndices(model, primitive.indices, vertexCount);

        if (result.indices.size() > kMaxMeshValue ||
            localIndices.size() > kMaxMeshValue - result.indices.size())
        {
            throw std::runtime_error("glTF mesh index range exceeds uint32_t");
        }

        const std::uint32_t indexStart =
            static_cast<std::uint32_t>(result.indices.size());

        result.indices.reserve(result.indices.size() + localIndices.size());

        for (const std::uint32_t localIndex : localIndices)
        {
            result.indices.push_back(baseVertex + localIndex);
        }

        SubMeshInfo subMesh;
        subMesh.indexStart = indexStart;
        subMesh.indexCount = static_cast<std::uint32_t>(localIndices.size());
        subMesh.defaultMaterialIndex = primitive.material;
        result.subMeshes.push_back(subMesh);
    }

    return result;
}

// Node는 모델의 한 부품과 부모·자식 관계를 나타낸다. 부모 기준 위치·회전·크기로 변환을 풀어 저장하고 회전은 길이가 1인 quaternion으로 맞춘다.
void ReadNodeTransform(
    const tinygltf::Node& source,
    NodeData& destination)
{
    constexpr float kTransformTolerance = 1.0e-5F;
    const bool hasMatrix = !source.matrix.empty();
    if (hasMatrix)
    {
        if (source.matrix.size() != 16 ||
            !source.translation.empty() ||
            !source.rotation.empty() ||
            !source.scale.empty())
        {
            throw std::runtime_error("glTF node must use either matrix or TRS");
        }

        glm::mat4 matrix{1.0F};

        // 행렬은 좌표를 다른 기준으로 바꾸는 숫자 표다. 파일과 GLM은 열을 먼저 나열하므로 같은 열·행 위치에 숫자를 복사한다.
        for (int column = 0; column < 4; ++column)
        {
            for (int row = 0; row < 4; ++row)
            {
                const double value = source.matrix[column * 4 + row];
                if (!std::isfinite(value) ||
                    std::abs(value) > std::numeric_limits<float>::max())
                {
                    throw std::runtime_error("glTF node matrix contains a non-finite value");
                }
                matrix[column][row] = static_cast<float>(value);
            }
        }

        // 원근은 멀리 있는 점을 작게 보이게 하는 효과다. 이런 원근 성분은 위치·회전·크기로 나타낼 수 없으므로 원근이 없는 변환만 받는다.
        if (std::abs(matrix[0][3]) > kTransformTolerance ||
            std::abs(matrix[1][3]) > kTransformTolerance ||
            std::abs(matrix[2][3]) > kTransformTolerance ||
            std::abs(matrix[3][3] - 1.0F) > kTransformTolerance)
        {
            throw std::runtime_error("Perspective node matrices are not supported");
        }

        glm::vec3 scale{1.0F};
        glm::quat orientation{};
        glm::vec3 translation{0.0F};
        glm::vec3 skew{0.0F};
        glm::vec4 perspective{0.0F};

        if (!glm::decompose(
                matrix,
                scale,
                orientation,
                translation,
                skew,
                perspective))
        {
            throw std::runtime_error("Failed to decompose glTF node matrix");
        }

        RequireFinite(scale);
        RequireFinite(translation);
        RequireFinite(orientation);
        RequireFinite(skew);
        RequireFinite(perspective);
        // Shear는 한 축을 다른 축 방향으로 기울이는 변환이다. 위치·회전·크기 세 값으로 보존할 수 없어 분해 결과에 shear가 있는지 확인한다.
        if (std::abs(scale.x) <= kTransformTolerance ||
            std::abs(scale.y) <= kTransformTolerance ||
            std::abs(scale.z) <= kTransformTolerance ||
            glm::length(orientation) <= kTransformTolerance ||
            glm::length(skew) > kTransformTolerance ||
            std::abs(perspective.x) > kTransformTolerance ||
            std::abs(perspective.y) > kTransformTolerance ||
            std::abs(perspective.z) > kTransformTolerance ||
            std::abs(perspective.w - 1.0F) > kTransformTolerance)
        {
            throw std::runtime_error("glTF node matrix must contain non-degenerate TRS only");
        }

        destination.translation = translation;
        destination.rotation = glm::normalize(orientation);
        destination.scale = scale;
        RequireFinite(destination.rotation);
        return;
    }

    if ((!source.translation.empty() && source.translation.size() != 3) ||
        (!source.rotation.empty() && source.rotation.size() != 4) ||
        (!source.scale.empty() && source.scale.size() != 3))
    {
        throw std::runtime_error("Invalid glTF node TRS component size");
    }

    if (source.translation.size() == 3)
    {
        for (const double value : source.translation)
        {
            if (!std::isfinite(value) ||
                std::abs(value) > std::numeric_limits<float>::max())
            {
                throw std::runtime_error("glTF node translation contains a non-finite value");
            }
        }
        destination.translation = glm::vec3{
            static_cast<float>(source.translation[0]),
            static_cast<float>(source.translation[1]),
            static_cast<float>(source.translation[2])};
    }

    if (source.scale.size() == 3)
    {
        for (const double value : source.scale)
        {
            if (!std::isfinite(value) ||
                std::abs(value) > std::numeric_limits<float>::max())
            {
                throw std::runtime_error("glTF node scale contains a non-finite value");
            }
        }
        destination.scale = glm::vec3{
            static_cast<float>(source.scale[0]),
            static_cast<float>(source.scale[1]),
            static_cast<float>(source.scale[2])};
    }

    if (source.rotation.size() == 4)
    {
        // Quaternion은 회전을 네 숫자로 나타내며 yaw/pitch 같은 세 축 회전각과 다른 표현이다. 파일의 [x,y,z,w] 순서를 GLM 생성자의 (w,x,y,z)로 옮긴다.
        for (const double value : source.rotation)
        {
            if (!std::isfinite(value) ||
                std::abs(value) > std::numeric_limits<float>::max())
            {
                throw std::runtime_error("glTF node rotation contains a non-finite value");
            }
        }
        const glm::quat quaternion{
            static_cast<float>(source.rotation[3]),
            static_cast<float>(source.rotation[0]),
            static_cast<float>(source.rotation[1]),
            static_cast<float>(source.rotation[2])};

        const double quaternionLength = std::sqrt(
            static_cast<double>(quaternion.x) * quaternion.x +
            static_cast<double>(quaternion.y) * quaternion.y +
            static_cast<double>(quaternion.z) * quaternion.z +
            static_cast<double>(quaternion.w) * quaternion.w);
        if (!std::isfinite(quaternionLength) || quaternionLength <= kTransformTolerance)
            throw std::runtime_error("glTF node rotation quaternion is zero");

        const glm::quat normalizedQuaternion{
            static_cast<float>(quaternion.w / quaternionLength),
            static_cast<float>(quaternion.x / quaternionLength),
            static_cast<float>(quaternion.y / quaternionLength),
            static_cast<float>(quaternion.z / quaternionLength)};
        // 길이가 0인 quaternion은 회전축과 회전량을 정할 수 없으므로 거부한다. 나머지는 회전 방향을 유지하며 길이를 1로 맞춘다.
        destination.rotation = normalizedQuaternion;
    }

    RequireFinite(destination.translation);
    RequireFinite(destination.rotation);
    RequireFinite(destination.scale);
    if (std::abs(destination.scale.x) <= kTransformTolerance ||
        std::abs(destination.scale.y) <= kTransformTolerance ||
        std::abs(destination.scale.z) <= kTransformTolerance)
    {
        throw std::runtime_error("glTF node scale must be non-degenerate");
    }
}
} // namespace

ModelResource GltfLoader::LoadGLB(const std::filesystem::path& path)
{
    tinygltf::TinyGLTF loader;
    tinygltf::Model gltfModel;

    std::string error;
    std::string warning;

    const bool loaded = loader.LoadBinaryFromFile(
        &gltfModel,
        &error,
        &warning,
        path.string());

    if (!loaded)
        throw std::runtime_error(
            "Failed to load GLB: " + path.string() + "\n" + error);

    /*
        TinyGLTF가 남긴 경고는 파일 읽기 실패와는 다르다.
        현재는 경고를 저장하거나 출력하는 기능이 없어 버린다. 진단 로그가 추가되면 Logger에 전달해야 한다.
    */
    (void)warning;

    ModelResource result;

    /*
        ResourceID는 캐시에서 자원을 다시 찾는 식별자다. 파일 경로와 자원 종류, 항목 번호를 조합하므로 서로 다른 GLB의 같은 번호도 구분된다.
        예: robot.glb#mesh/0은 첫 형상, robot.glb#material/0은 첫 재질, robot.glb#texture/0은 첫 glTF texture 항목이다.
    */
    const std::string resourcePrefix = path.generic_string();

    result.textures.reserve(gltfModel.textures.size());
    for (std::size_t i = 0; i < gltfModel.textures.size(); ++i)
    {
        result.textures.push_back(
            ConvertTexture(
                gltfModel,
                gltfModel.textures[i],
                resourcePrefix,
                i));
    }

    result.materials.reserve(gltfModel.materials.size());
    for (std::size_t i = 0; i < gltfModel.materials.size(); ++i)
    {
        result.materials.push_back(
            ConvertMaterial(
                gltfModel.materials[i],
                resourcePrefix,
                i));
    }

    result.meshes.reserve(gltfModel.meshes.size());
    for (std::size_t i = 0; i < gltfModel.meshes.size(); ++i)
    {
        result.meshes.push_back(
            ConvertMesh(
                gltfModel,
                gltfModel.meshes[i],
                resourcePrefix,
                i));
    }

    result.nodes.resize(gltfModel.nodes.size());
    for (std::size_t i = 0; i < gltfModel.nodes.size(); ++i)
    {
        const tinygltf::Node& source = gltfModel.nodes[i];
        NodeData& destination = result.nodes[i];

        destination.name = source.name.empty()
            ? "Node_" + std::to_string(i)
            : source.name;

        if (source.mesh < -1 || source.mesh >= static_cast<int>(result.meshes.size()))
            throw std::runtime_error("Invalid glTF node mesh index");
        destination.meshIndex = source.mesh;
        destination.childrenIndices = source.children;
        ReadNodeTransform(source, destination);
    }

    /*
        glTF 부품은 자식 번호만 기록하고 부모 번호는 기록하지 않는다.
        PrefabFactory가 각 부품의 부모를 알 수 있도록 자식 목록을 훑어 parentIndex를 채운다.
        두 부모가 같은 부품을 자식으로 가리키면 한 갈래 계층으로 만들 수 없어 거부한다.
    */
    for (std::size_t parentIndex = 0;
         parentIndex < result.nodes.size();
         ++parentIndex)
    {
        const NodeData& parent = result.nodes[parentIndex];

        for (const int childIndex : parent.childrenIndices)
        {
            if (childIndex < 0 ||
                childIndex >= static_cast<int>(result.nodes.size()))
            {
                throw std::runtime_error("Invalid child node index");
            }

            NodeData& child = result.nodes[childIndex];

            if (child.parentIndex != -1)
                throw std::runtime_error("glTF node has multiple parents");

            child.parentIndex = static_cast<int>(parentIndex);
        }
    }

    // 부모 연결을 따라가며 순환을 검사한다. 현재 방문 중인 부품으로 되돌아오면 잘못된 계층이므로 거부한다.
    std::vector<std::uint8_t> nodeState(result.nodes.size(), 0U);
    for (std::size_t start = 0; start < result.nodes.size(); ++start)
    {
        int current = static_cast<int>(start);
        while (current >= 0 && nodeState[static_cast<std::size_t>(current)] == 0U)
        {
            nodeState[static_cast<std::size_t>(current)] = 1U;
            current = result.nodes[static_cast<std::size_t>(current)].parentIndex;
        }

        if (current >= 0 && nodeState[static_cast<std::size_t>(current)] == 1U)
            throw std::runtime_error("glTF node hierarchy contains a cycle");

        current = static_cast<int>(start);
        while (current >= 0 && nodeState[static_cast<std::size_t>(current)] == 1U)
        {
            nodeState[static_cast<std::size_t>(current)] = 2U;
            current = result.nodes[static_cast<std::size_t>(current)].parentIndex;
        }
    }

    // 파일이 기본 장면을 지정하지 않았으면 첫 장면을 사용한다.
    int sceneIndex = gltfModel.defaultScene;
    if (sceneIndex < 0 && !gltfModel.scenes.empty())
        sceneIndex = 0;

    if (sceneIndex >= static_cast<int>(gltfModel.scenes.size()))
        throw std::runtime_error("Invalid glTF default scene index");

    if (sceneIndex >= 0 &&
        sceneIndex < static_cast<int>(gltfModel.scenes.size()))
    {
        const tinygltf::Scene& scene = gltfModel.scenes[sceneIndex];

        if (scene.nodes.size() > 1)
            throw std::runtime_error("Multiple root nodes in glTF scenes are not supported");
        if (scene.nodes.size() == 1)
        {
            const int rootIndex = scene.nodes.front();
            if (rootIndex < 0 || rootIndex >= static_cast<int>(result.nodes.size()) ||
                result.nodes[static_cast<std::size_t>(rootIndex)].parentIndex != -1)
            {
                throw std::runtime_error("Invalid glTF scene root node");
            }
            result.rootNodeIndex = rootIndex;
        }
    }

    // 장면에 최상위 부품이 지정되지 않았으면 부모가 없는 부품을 찾는다. 둘 이상이면 어느 장면인지 정할 수 없어 거부한다.
    if (result.rootNodeIndex < 0)
    {
        int rootCandidate = -1;
        bool multipleRoots = false;

        for (std::size_t i = 0; i < result.nodes.size(); ++i)
        {
            if (result.nodes[i].parentIndex != -1) continue;

            if (rootCandidate != -1)
            {
                multipleRoots = true;
                continue;
            }

            rootCandidate = static_cast<int>(i);
        }

        if (multipleRoots)
            throw std::runtime_error("Multiple root nodes in glTF scenes are not supported");
        result.rootNodeIndex = rootCandidate;
    }

    // PrefabFactory가 파일의 모든 부품을 만들기 때문에 선택한 장면과 떨어진 별도 부품 계층이 있으면 거부한다.
    for (std::size_t i = 0; i < result.nodes.size(); ++i)
    {
        if (result.nodes[i].parentIndex == -1 &&
            static_cast<int>(i) != result.rootNodeIndex)
        {
            throw std::runtime_error("glTF nodes must form a single tree under the scene root");
        }
    }

    return result;
}
