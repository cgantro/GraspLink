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
// Accessor -> BufferView -> Buffer를 따라 범위를 확인한 뒤 읽기 시작 주소와 간격을 보관한다.
struct AccessorView
{
    const unsigned char* data = nullptr;
    std::size_t stride = 0;
    std::size_t count = 0;
};

// 범위 검증 후 byte 주소를 만든다. Sparse 데이터는 현재 지원하지 않는다.
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
        ByteStride는 tightly packed와 interleaved 배열 모두에서 다음 원소의 시작점을 준다.
        예: [Pos][Normal][UV]가 반복되면 POSITION의 간격은 POSITION 자체 크기보다 크다.
        아래 범위 계산은 마지막 원소의 끝까지 포함하므로 padding이 bufferView 밖으로
        이어지는 malformed 데이터도 포인터를 만들기 전에 거부한다.
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

// FLOAT VEC3를 Position/Normal 배열로 복사한다.
std::vector<glm::vec3> ReadVec3FloatAccessor(
    const tinygltf::Model& model,
    int accessorIndex)
{
    const tinygltf::Accessor& accessor = model.accessors.at(accessorIndex);
    if (accessor.type != TINYGLTF_TYPE_VEC3 ||
        accessor.componentType != TINYGLTF_COMPONENT_TYPE_FLOAT)
    {
        throw std::runtime_error("Expected FLOAT VEC3 ACCESSOR");
    }

    constexpr std::size_t kElementSize = sizeof(float) * 3U;
    const AccessorView view = GetAccessorView(model, accessorIndex, kElementSize);

    std::vector<glm::vec3> result(view.count);

    for (std::size_t i = 0; i < view.count; ++i)
    {
        const unsigned char* source = view.data + i * view.stride;
        float values[3]{};

        // Buffer 주소의 float alignment를 가정하지 않기 위해 reinterpret_cast 대신 memcpy를 사용한다.
        std::memcpy(values, source, sizeof(values));
        result[i] = glm::vec3{values[0], values[1], values[2]};
        RequireFinite(result[i]);
    }

    return result;
}

// FLOAT VEC2를 UV 배열로 복사한다.
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

// 현재 Vertex는 tangent xyz만 보관한다. Normal mapping에는 w 보존도 필요하다.
std::vector<glm::vec3> ReadTangentAccessor(
    const tinygltf::Model& model,
    int accessorIndex)
{
    const tinygltf::Accessor& accessor = model.accessors.at(accessorIndex);

    if (accessor.type != TINYGLTF_TYPE_VEC4 ||
        accessor.componentType != TINYGLTF_COMPONENT_TYPE_FLOAT)
    {
        throw std::runtime_error("Expected FLOAT VEC4 tangent accessor");
    }

    constexpr std::size_t kElementSize = sizeof(float) * 4U;
    const AccessorView view = GetAccessorView(model, accessorIndex, kElementSize);

    std::vector<glm::vec3> result(view.count);

    for (std::size_t i = 0; i < view.count; ++i)
    {
        const unsigned char* source = view.data + i * view.stride;
        float values[4]{};
        std::memcpy(values, source, sizeof(values));
        result[i] = glm::vec3{values[0], values[1], values[2]};
        RequireFinite(result[i]);
    }

    return result;
}

// glTF byte 주소가 T의 정렬 경계에 놓인다고 가정하지 않고 scalar를 복사한다.
// 접근자의 파일 형식상 정렬 요구를 검사하는 코드는 별도 없다.
template<typename T>
T ReadScalar(const unsigned char* data)
{
    T value{};
    std::memcpy(&value, data, sizeof(T));
    return value;
}

// Index는 uint32_t로 통일한다. 없으면 정점 순서로 생성한다.
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

    std::size_t componentSize = 0;

    switch (accessor.componentType)
    {
    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
        componentSize = sizeof(std::uint8_t);
        break;
    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
        componentSize = sizeof(std::uint16_t);
        break;
    case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
        componentSize = sizeof(std::uint32_t);
        break;
    default:
        throw std::runtime_error("Unsupported glTF index component type");
    }

    const AccessorView view = GetAccessorView(model, accessorIndex, componentSize);
    std::vector<std::uint32_t> result;
    result.reserve(view.count);

    for (std::size_t i = 0; i < view.count; ++i)
    {
        const unsigned char* source = view.data + i * view.stride;
        std::uint32_t value = 0;

        switch (accessor.componentType)
        {
        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
            value = ReadScalar<std::uint8_t>(source);
            break;
        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
            value = ReadScalar<std::uint16_t>(source);
            break;
        case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
            value = ReadScalar<std::uint32_t>(source);
            break;
        default:
            throw std::runtime_error("Unsupported glTF index type");
        }

        if (value >= vertexCount)
            throw std::runtime_error("glTF index exceeds vertex Count");

        result.push_back(value);
    }

    return result;
}

// TinyGLTF 디코딩이 끝난 8-bit RGB/RGBA 픽셀을 CPU 데이터와 안정적인 Texture ID로 복사한다.
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

// PBR factor와 base color texture를 내부 Material에 연결한다.
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

    if (source.emissiveFactor.size() == 3)
    {
        result.emissiveFactor = glm::vec3{
            static_cast<float>(source.emissiveFactor[0]),
            static_cast<float>(source.emissiveFactor[1]),
            static_cast<float>(source.emissiveFactor[2])};
    }

    // glTF Material은 Texture index를 보관하지만 내부 IR은 파일 경로 기반 ResourceID로 연결한다.
    if (pbr.baseColorTexture.index >= 0)
    {
        result.baseColorTexture = ResourceID{
            resourcePrefix + "#texture/" +
            std::to_string(pbr.baseColorTexture.index)};
    }

    return result;
}

// Primitive별 local index에 baseVertex를 더해 하나의 Mesh로 합친다.
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

        // POSITION은 렌더 가능한 Primitive에 필수다.
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

        // NORMAL/TEXCOORD/TANGENT는 선택 attribute다. 없으면 Vertex 기본값을 사용한다.
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

        std::vector<glm::vec3> tangents;
        const auto tangentIterator = primitive.attributes.find("TANGENT");
        if (tangentIterator != primitive.attributes.end())
        {
            tangents = ReadTangentAccessor(model, tangentIterator->second);
            if (tangents.size() != vertexCount)
                throw std::runtime_error("TANGENT count does not match POSITION count");
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
            if (!tangents.empty()) vertex.tangent = tangents[i];
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

// 부모 기준 matrix/TRS를 NodeData의 Local TRS로 바꾼다. 회전 출력 단위는 Euler radian이다.
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

        // glTF matrix는 column-major다. GLM 행렬의 같은 [column][row] 위치에 복사한다.
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

        // 마지막 행이 원근 성분을 가지면 TRS로 표현할 수 없어 허용 오차 안의 affine만 받는다.
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
        // Euler/TRS 결과에는 shear를 저장할 자리가 없으므로 분해 결과가 순수 TRS인지 확인한다.
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
        // 결과 NodeData는 quaternion이 아니라 Euler radian을 저장한다. 회전 표현 정밀도는 이 변환을 따른다.
        destination.rotation = glm::eulerAngles(glm::normalize(orientation));
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
        // glTF wire 순서 [x,y,z,w]를 GLM 생성자 인자 순서 (w,x,y,z)로 옮긴다.
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
        // 잘못된 길이의 quaternion은 먼저 거부하고 단위 quaternion으로 만든 뒤 Euler radian으로 보관한다.
        destination.rotation = glm::eulerAngles(normalizedQuaternion);
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
        TinyGLTF warning은 load failure가 아니다.
        현재 Logger가 없어서 버리지만 진단 정보가 필요해지면 warning을 Logger에 전달한다.
    */
    (void)warning;

    ModelResource result;

    /*
        파일 경로를 Resource namespace prefix로 사용한다.
        같은 mesh index라도 파일이 다르면 ID가 달라진다.

        예:
        HCR12A_R00.glb#mesh/0
        HCR12A_R00.glb#material/0
        HCR12A_R00.glb#texture/0
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
        glTF Node는 children index만 저장하고 parent index는 직접 갖지 않는다.
        PrefabFactory가 parent를 빠르게 복원할 수 있도록 children 정보를 역으로 순회해 parentIndex를 채운다.
        두 부모가 같은 Node를 참조하면 트리 변환이 모호하므로 즉시 거부한다.
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

    // 부모가 하나인 그래프를 세 상태로 방문해, 이미 진행 중인 경로로 돌아오는 순환을 거부한다.
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

    // defaultScene이 없고 Scene이 하나 이상이면 첫 Scene을 fallback으로 사용한다.
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

    // Scene root가 없으면 parent가 없는 Node를 찾고, 둘 이상이면 거부한다.
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

    // PrefabFactory는 모든 Node를 생성하므로 선택한 Scene 밖의 별도 트리는 거부한다.
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
