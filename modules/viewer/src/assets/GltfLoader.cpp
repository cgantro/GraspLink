#include "assets/GltfLoader.h"

#include <tiny_gltf.h>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/quaternion.hpp>

#include <cstdint>
#include <cstring>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
/**
 * @brief glTF Accessor가 가리키는 실제 byte 영역을 읽기 쉽게 정리한 내부 view.
 *
 * @details
 * glTF binary data 접근 경로는 `Accessor -> BufferView -> Buffer` 순서다.
 * 실제 첫 element 주소는 다음과 같다.
 *
 * `buffer.data + bufferView.byteOffset + accessor.byteOffset`
 *
 * stride는 interleaved vertex buffer에서도 다음 element 위치를 올바르게 찾기 위해 필요하다.
 */
struct AccessorView
{
    const unsigned char* data = nullptr;
    std::size_t stride = 0;
    std::size_t count = 0;
};

/**
 * @brief glTF Accessor의 byte 시작 주소, stride, element count를 검증해서 얻는다.
 * @param model TinyGLTF가 파싱한 전체 model.
 * @param accessorIndex model.accessors index.
 * @param elementSize 호출자가 기대하는 element 하나의 최소 byte 크기.
 * @return 검증된 AccessorView.
 * @throws std::runtime_error 잘못된 index, BufferView 없는 accessor, buffer 범위 초과인 경우.
 *
 * @todo [FUTURE] sparse accessor가 필요한 asset을 지원할 때 BufferView 없는 sparse case를 별도로 처리한다.
 */
AccessorView GetAccessorView(
    const tinygltf::Model& model,
    int accessorIndex,
    std::size_t elementSize)
{
    if (accessorIndex < 0 || accessorIndex >= static_cast<int>(model.accessors.size()))
        throw std::runtime_error("Invalid glTF accessor Index");

    const tinygltf::Accessor& accessor = model.accessors[accessorIndex];

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
        ByteStride는 tightly packed attribute와 interleaved attribute를 같은 방식으로 순회하게 해준다.
        예: [Pos][Normal][UV][Pos][Normal][UV]라면 다음 Pos까지의 간격은 Pos 크기보다 크다.
    */
    const int byteStride = accessor.ByteStride(bufferView);
    if (byteStride <= 0)
        throw std::runtime_error("Invalid glTF accessor stride");

    const std::size_t offset =
        static_cast<std::size_t>(bufferView.byteOffset) +
        static_cast<std::size_t>(accessor.byteOffset);

    if (accessor.count > 0)
    {
        const std::size_t lastByte =
            offset +
            static_cast<std::size_t>(byteStride) *
                (static_cast<std::size_t>(accessor.count) - 1U) +
            elementSize;

        if (lastByte > buffer.data.size())
            throw std::runtime_error("glTF accessor exceeds buffer size");
    }

    return AccessorView{
        buffer.data.data() + offset,
        static_cast<std::size_t>(byteStride),
        static_cast<std::size_t>(accessor.count)};
}

/**
 * @brief FLOAT VEC3 Accessor를 glm::vec3 배열로 복사한다.
 * @param model TinyGLTF model.
 * @param accessorIndex 읽을 Accessor index.
 * @return Position/Normal 등에 사용할 vec3 배열.
 */
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
    }

    return result;
}

/**
 * @brief FLOAT VEC2 Accessor를 glm::vec2 배열로 복사한다.
 * @param model TinyGLTF model.
 * @param accessorIndex 읽을 Accessor index.
 * @return UV(TEXCOORD_0) 등에 사용할 vec2 배열.
 */
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
    }

    return result;
}

/**
 * @brief glTF FLOAT VEC4 tangent에서 xyz만 읽어 현재 Vertex::tangent(vec3)로 변환한다.
 * @param model TinyGLTF model.
 * @param accessorIndex TANGENT Accessor index.
 * @return tangent xyz 배열.
 *
 * @warning glTF tangent의 w는 bitangent handedness다. 현재 구조에서는 버리고 있으므로
 *          normal mapping을 정확히 구현하기 전에는 tangent-space 계산에 사용하면 안 된다.
 * @todo [FUTURE] normal mapping 구현 시 Vertex::tangent를 vec4로 바꾸고 w handedness까지 보존한다.
 */
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
    }

    return result;
}

/**
 * @brief alignment 가정 없이 binary scalar 한 개를 타입 T로 복사한다.
 * @tparam T uint8_t/uint16_t/uint32_t 등 trivially copyable scalar 타입.
 * @param data scalar가 시작되는 byte 주소.
 * @return 복사된 scalar 값.
 */
template<typename T>
T ReadScalar(const unsigned char* data)
{
    T value{};
    std::memcpy(&value, data, sizeof(T));
    return value;
}

/**
 * @brief glTF index Accessor를 renderer 공통 형식인 uint32_t 배열로 변환한다.
 * @param model TinyGLTF model.
 * @param accessorIndex index Accessor index. 음수이면 non-indexed Primitive로 처리한다.
 * @param vertexCount 현재 Primitive의 vertex 개수. index 범위 검증에 사용한다.
 * @return uint32_t index 배열.
 *
 * @details glTF는 UNSIGNED_BYTE/SHORT/INT index를 허용하지만 내부 MeshData는 uint32_t로 통일한다.
 * index accessor가 없는 Primitive는 0,1,2,... 순차 index를 생성한다.
 */
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

/**
 * @brief glTF Texture가 참조하는 decoded Image를 TextureData로 복사한다.
 * @param model TinyGLTF model. source.source가 model.images index를 가리킨다.
 * @param source 변환할 glTF Texture.
 * @param resourcePrefix 같은 파일 내 resource namespace 역할을 하는 파일 경로 문자열.
 * @param textureIndex model.textures index.
 * @return CPU pixel과 ResourceID를 가진 TextureData.
 *
 * @note 현재 GPU Texture wrapper가 지원하는 범위에 맞춰 8-bit RGB/RGBA만 허용한다.
 * @todo [FUTURE] glTF sampler의 filter/wrap 정보도 TextureData 또는 별도 SamplerData로 보존한다.
 */
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

/**
 * @brief glTF PBR metallic-roughness Material을 renderer 독립적인 MaterialData로 변환한다.
 * @param source 변환할 glTF Material.
 * @param resourcePrefix ResourceID namespace용 파일 경로 prefix.
 * @param materialIndex model.materials index.
 * @return factor와 texture ResourceID를 가진 MaterialData.
 *
 * @todo [FUTURE] metallicRoughness/normal/occlusion/emissive texture와 alphaMode/alphaCutoff를 연결한다.
 */
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

/**
 * @brief 하나의 glTF Mesh와 그 Primitive들을 하나의 MeshData + SubMeshInfo 배열로 변환한다.
 * @param model Accessor/Buffer를 읽기 위한 TinyGLTF model.
 * @param source 변환할 glTF Mesh.
 * @param resourcePrefix ResourceID namespace용 파일 경로 prefix.
 * @param meshIndex model.meshes index.
 * @return 합쳐진 vertex/index 배열과 Primitive별 draw range를 가진 MeshData.
 *
 * @details
 * Primitive마다 vertex index가 0부터 시작하므로 여러 Primitive를 하나의 MeshData buffer로 합칠 때
 * `baseVertex`를 각 local index에 더해 전체 vertex 배열 기준 index로 보정한다.
 */
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

/**
 * @brief glTF Node의 matrix 또는 TRS 표현을 NodeData의 Local TRS로 변환한다.
 * @param source TinyGLTF Node.
 * @param destination 결과를 기록할 NodeData.
 *
 * @details
 * glTF Node는 4x4 matrix 하나 또는 translation/rotation/scale을 사용할 수 있다.
 * matrix 방식은 glm::decompose로 TRS를 분리하고, quaternion rotation은 현재 ECS 표현에 맞춰 Euler radian으로 바꾼다.
 *
 * @todo [FUTURE] ECS Rotation을 quaternion으로 바꾸면 quaternion -> Euler 변환을 제거해 회전 정보를 그대로 보존한다.
 */
void ReadNodeTransform(
    const tinygltf::Node& source,
    NodeData& destination)
{
    if (source.matrix.size() == 16)
    {
        glm::mat4 matrix{1.0F};

        // glTF와 GLM 모두 column-major convention을 사용한다.
        for (int column = 0; column < 4; ++column)
        {
            for (int row = 0; row < 4; ++row)
            {
                matrix[column][row] =
                    static_cast<float>(source.matrix[column * 4 + row]);
            }
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

        destination.translation = translation;
        destination.rotation = glm::eulerAngles(glm::normalize(orientation));
        destination.scale = scale;
        return;
    }

    if (source.translation.size() == 3)
    {
        destination.translation = glm::vec3{
            static_cast<float>(source.translation[0]),
            static_cast<float>(source.translation[1]),
            static_cast<float>(source.translation[2])};
    }

    if (source.scale.size() == 3)
    {
        destination.scale = glm::vec3{
            static_cast<float>(source.scale[0]),
            static_cast<float>(source.scale[1]),
            static_cast<float>(source.scale[2])};
    }

    if (source.rotation.size() == 4)
    {
        // glTF quaternion array는 [x,y,z,w], GLM constructor는 (w,x,y,z) 순서다.
        const glm::quat quaternion{
            static_cast<float>(source.rotation[3]),
            static_cast<float>(source.rotation[0]),
            static_cast<float>(source.rotation[1]),
            static_cast<float>(source.rotation[2])};

        destination.rotation = glm::eulerAngles(glm::normalize(quaternion));
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

        destination.meshIndex = source.mesh;
        destination.childrenIndices = source.children;
        ReadNodeTransform(source, destination);
    }

    /*
        glTF Node는 children index만 저장하고 parent index는 직접 갖지 않는다.
        PrefabFactory가 parent를 빠르게 복원할 수 있도록 children 정보를 역으로 순회해 parentIndex를 채운다.
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

    // defaultScene이 없고 Scene이 하나 이상이면 첫 Scene을 fallback으로 사용한다.
    int sceneIndex = gltfModel.defaultScene;
    if (sceneIndex < 0 && !gltfModel.scenes.empty())
        sceneIndex = 0;

    if (sceneIndex >= 0 &&
        sceneIndex < static_cast<int>(gltfModel.scenes.size()))
    {
        const tinygltf::Scene& scene = gltfModel.scenes[sceneIndex];

        // 현재 ModelResource는 단일 root만 표현한다.
        if (scene.nodes.size() == 1)
            result.rootNodeIndex = scene.nodes.front();
    }

    /*
        Scene metadata로 단일 root를 결정하지 못하면 parent가 없는 Node를 직접 찾는다.
        후보가 둘 이상이면 multi-root이므로 단일 root를 임의 선택하지 않고 invalid(-1)로 남긴다.
    */
    if (result.rootNodeIndex < 0)
    {
        int rootCandidate = -1;

        for (std::size_t i = 0; i < result.nodes.size(); ++i)
        {
            if (result.nodes[i].parentIndex != -1) continue;

            if (rootCandidate != -1)
            {
                rootCandidate = -1;
                break;
            }

            rootCandidate = static_cast<int>(i);
        }

        result.rootNodeIndex = rootCandidate;
    }

    // TODO(FUTURE): 범용 glTF 지원이 필요하면 ModelResource::rootNodeIndex를 rootNodeIndices로 확장한다.
    return result;
}
