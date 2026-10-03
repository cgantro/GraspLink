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
 * @brief Accessor
 * 
 * glTF 데이터 접근 구조 : Accessor -> BufferView -> Buffer
 * 
 * 실제 데이터 시작 주소:
 *      buffer.data + bufferView.byteOffset + accessor.byteOffset
 */
struct AccessorView{
    const unsigned char* data = nullptr;
    // 다음 element까지 이동할 byte 수
    std::size_t stride = 0; 
    // element 개수
    std::size_t count = 0;
};

/**
 * @brief Get the Accessor View object
 * 
 * @param model 
 * @param accessorIndex 
 * @param elementSize 
 * @return AccessorView 
 */
AccessorView GetAccessorView(const tinygltf::Model& model, int accessorIndex, std::size_t elementSize){
    if(accessorIndex < 0 || accessorIndex >= static_cast<int>(model.accessors.size())) 
        throw std::runtime_error("Invalid glTF accessor Index");

    const tinygltf::Accessor& accessor = model.accessors[accessorIndex];
    /*
        Sparse Accessor는 BufferView없이 존재 가능
        현재 프로젝트에서는 일반적인 BufferView 기반 GLB만 처리
    */
    if(accessor.bufferView < 0 || accessor.bufferView >=  static_cast<int>(model.bufferViews.size()))
        throw std::runtime_error("Accessor without BufferView is not supported");

    const tinygltf::BufferView& bufferView = model.bufferViews[accessor.bufferView];
    if (bufferView.buffer < 0 || bufferView.buffer >= static_cast<int>(model.buffers.size()))
        throw std::runtime_error("Invalid glTF buffer index");

    const tinygltf::Buffer& buffer = model.buffers[bufferView.buffer];
    /*
        Accessor::ByteStride()
        InterLeaved Vertex Buffer인지,
        attribute별 독립 Buffer인지에 따라 Stride 계산
    */
    const int byteStride = accessor.ByteStride(bufferView);
    if(byteStride <= 0) throw std::runtime_error("Invalid glTF accessor stride");

    const std::size_t offset = 
        static_cast<std::size_t>(bufferView.byteOffset) +
        static_cast<std::size_t>(accessor.byteOffset);

    // 마지막 Element까지 Buffer 범위 내부인지 검증
    if(accessor.count > 0){
        const std::size_t lastByte = 
            offset + static_cast<std::size_t>(byteStride) * (static_cast<std::size_t>(accessor.count) - 1U) + elementSize;
        
        if(lastByte > buffer.data.size()) throw std::runtime_error("glTF accessor exceeds buffer size");
    }

    return AccessorView{
        buffer.data.data() + offset,
        static_cast<std::size_t>(byteStride),
        static_cast<std::size_t>(accessor.count)
    };
}

/**
 * @brief 
 * 
 * @param model 
 * @param accessorIndex 
 * @return std::vector<glm::vec3> 
 */
std::vector<glm::vec3> ReadVec3FloatAccessor(const tinygltf::Model& model, int accessorIndex){
    const tinygltf::Accessor& accessor = model.accessors.at(accessorIndex);
    if(accessor.type != TINYGLTF_TYPE_VEC3 || accessor.componentType != TINYGLTF_COMPONENT_TYPE_FLOAT)
        throw std::runtime_error("Expected FLOAT  VEC3 ACCESSOR");
    
    constexpr std::size_t kElementSize = sizeof(float) * 3U;

    const AccessorView view = GetAccessorView(model,accessorIndex,kElementSize);
    
    std::vector<glm::vec3> result;
    result.resize(view.count);

    for(std::size_t i = 0; i< view.count; i++){
        const unsigned char* source = view.data + i * view.stride;

        float values[3]{};
        /*
            직접 float*로 reinterpret_cast 하지 않고, memcpy 사용
            Buffer 내부 주소 alignment에 대한 불필요한 가정 하지 않기 위함
        */
        std::memcpy(values, source, sizeof(values));
        result[i] = glm::vec3{values[0],values[1],values[2]};
    }
    return result;
}

/**
 * @brief 
 * 
 * @param model 
 * @param accessorIndex 
 * @return std::vector<glm::vec2> 
 */
std::vector<glm::vec2> ReadVec2FloatAccessor(const tinygltf::Model& model, int accessorIndex){
    const tinygltf::Accessor& accessor = model.accessors.at(accessorIndex);
    if(accessor.type != TINYGLTF_TYPE_VEC2 || accessor.componentType != TINYGLTF_COMPONENT_TYPE_FLOAT)
        throw std::runtime_error("Expected FLOAT VEC2 ACCESSOR");

    constexpr std::size_t kElementSize = sizeof(float) * 2U;

    const AccessorView view = GetAccessorView(model,accessorIndex,kElementSize);
    
    std::vector<glm::vec2> result;
    result.resize(view.count);

    for(std::size_t i = 0; i< view.count; i++){
        const unsigned char* source = view.data + i * view.stride;

        float values[2]{};
        /*
            직접 float*로 reinterpret_cast 하지 않고, memcpy 사용
            Buffer 내부 주소 alignment에 대한 불필요한 가정 하지 않기 위함
        */
        std::memcpy(values, source, sizeof(values));
        result[i] = glm::vec2{values[0],values[1]};
    }
    return result;
}

/**
 * @brief 
 * 
 * @param model 
 * @param accessorIndex 
 * @return std::vector<glm::vec3> 
 */
std::vector<glm::vec3> ReadTangentAccessor(
    const tinygltf::Model& model,
    int accessorIndex)
{
    const tinygltf::Accessor& accessor =model.accessors.at(accessorIndex);

    /*
        glTF tangent는 VEC4이다.

        xyz = tangent vector
        w   = handedness

        현재 Vertex 구조에는 tangent vec3만 있으므로
        xyz만 저장한다.

        나중에 Normal Mapping을 구현할 때는
        Vertex::tangent를 glm::vec4로 바꾸는 것이 맞다.
    */
    if (accessor.type != TINYGLTF_TYPE_VEC4 || accessor.componentType != TINYGLTF_COMPONENT_TYPE_FLOAT)
        throw std::runtime_error("Expected FLOAT VEC4 tangent accessor");

    constexpr std::size_t kElementSize = sizeof(float) * 4U;

    const AccessorView view =GetAccessorView(model,accessorIndex,kElementSize);

    std::vector<glm::vec3> result;
    result.resize(view.count);

    for (std::size_t i = 0; i < view.count; ++i)
    {
        const unsigned char* source = view.data + i * view.stride;

        float values[4]{};

        std::memcpy(values,source,sizeof(values));

        result[i] = glm::vec3{values[0],values[1],values[2]};
    }

    return result;
}

/*
    Index
*/

/**
 * @brief 
 * 
 * @tparam T 
 * @param data 
 * @return T 
 */
template<typename T>
T ReadScalar(const unsigned char* data){
    T value{};
    std::memcpy(&value, data, sizeof(T));
    return value;
}

/**
 * @brief 
 * 
 * @param model 
 * @param accessorIndex 
 * @param vertexCount 
 * @return std::vector<std::uint32_t> 
 */
std::vector<std::uint32_t> ReadIndices(const tinygltf::Model& model, int accessorIndex, std::size_t vertexCount){
    /*
        glTF Primitive는 Index Buffer가 없을 수도 있음
        그 경우, Vertex 0,1,2... 순서 자체를 Idx로 사용한다.
    */
    if(accessorIndex < 0){
        std::vector<std::uint32_t> indices(vertexCount);
        std::iota(indices.begin(),indices.end(), 0U); // 컨테이너나 배열의 범위를 1씩 증가하는 연속된 숫자로 채우는 함수
        return indices;
    }

    const tinygltf::Accessor& accessor = model.accessors.at(accessorIndex);
    if(accessor.type != TINYGLTF_TYPE_SCALAR) throw std::runtime_error("Index Accessor must be SCALAR");

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

    const AccessorView view = GetAccessorView(model,accessorIndex,componentSize);
    std::vector<std::uint32_t> result;
    result.reserve(view.count);

    for(std::size_t i = 0; i < view.count; i++){
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

        if(value >= vertexCount) throw std::runtime_error("glTF index exceeds vertex Count");
        result.push_back(value);
    }
    return result;
}
/*
    Texture
*/

TextureData ConvertTexture(
    const tinygltf::Model& model,
    const tinygltf::Texture& source,
    const std::string& resourcePrefix,
    std::size_t textureIndex)
{
    TextureData result;

    result.uniqueID = ResourceID{resourcePrefix +"#texture/" +std::to_string(textureIndex)};

    if (source.source < 0 ||
        source.source >= static_cast<int>(model.images.size()))
        throw std::runtime_error( "Invalid glTF texture source");

    const tinygltf::Image& image =
        model.images[source.source];

    if (image.width <= 0 || image.height <= 0) throw std::runtime_error("Invalid glTF image size");

    if (image.component != 3 && image.component != 4) throw std::runtime_error("Only RGB/RGBA glTF images are supported");
    

    if (image.bits != 8) throw std::runtime_error("Only 8-bit glTF images are supported");

    if (image.image.empty()) throw std::runtime_error("glTF image contains no decoded pixels");

    result.width = image.width;
    result.height = image.height;
    result.channels = image.component;
    result.pixels = image.image;

    return result;
}
/*
    Material
*/

/**
 * @brief 
 * 
 * @param source 
 * @param resourcePrefix 
 * @param materialIndex 
 * @return MaterialData 
 */
MaterialData ConvertMaterial(
    const tinygltf::Material& source,
    const std::string& resourcePrefix,
    std::size_t materialIndex){
    MaterialData result;
    result.name = source.name;

    // 같은 파일 내부에서도 Material마다 ResourceID가 달라야 함
    result.uniqueID = ResourceID{resourcePrefix + "#material/" + std::to_string(materialIndex)};
    const auto& pbr = source.pbrMetallicRoughness;

    if(pbr.baseColorFactor.size() == 4){
        result.baseColorFactor = glm::vec4{
            static_cast<float>(pbr.baseColorFactor[0]),
            static_cast<float>(pbr.baseColorFactor[1]),
            static_cast<float>(pbr.baseColorFactor[2]),
            static_cast<float>(pbr.baseColorFactor[3])
        };
    }

    result.metallicFactor = static_cast<float>(pbr.metallicFactor);
    result.roughnessFactor = static_cast<float>(pbr.roughnessFactor);

    if(source.emissiveFactor.size() == 3){
        result.emissiveFactor =
            glm::vec3{
                static_cast<float>(source.emissiveFactor[0]),
                static_cast<float>(source.emissiveFactor[1]),
                static_cast<float>(source.emissiveFactor[2])
            };
    }

    /*
        Texture Resource는 아직 처리 X
        향후 GltfLoader -> Image/Texture IR -> AssetManager -> OpenGL Texture 계층이 생긴 뒤 추가.
    */
    /* 추가 완료*/
    if (pbr.baseColorTexture.index >= 0)
        result.baseColorTexture = ResourceID{resourcePrefix +"#texture/" +std::to_string(pbr.baseColorTexture.index)};
    
    return result;
}

/*
    Mesh
*/

/**
 * @brief 
 * 
 * @param model 
 * @param source 
 * @param resourcePrefix 
 * @param meshIndex 
 * @return MeshData 
 */
MeshData ConvertMesh(
    const tinygltf::Model& model,
    const tinygltf::Mesh& source,
    const std::string& resourcePrefix,
    std::size_t meshIndex){
    
    MeshData result;
    result.name = source.name;
    result.uniqueID = ResourceID{ resourcePrefix + "#mesh/" + std::to_string(meshIndex)};

    /*
        하나의 glTF Mesh는 여러 Prim을 가질 수 있음
        glTF:
            Mesh
            ├─ Primitive 0
            ├─ Primitive 1
            └─ Primitive 2

        내부 표현:
            MeshData
            ├─ vertices
            ├─ indices
            └─ SubMeshInfo[]
    */
    for(const tinygltf::Primitive& primitive: source.primitives){
        // 현재 렌더러는 GL_삼각형으로만 그림 
        if(primitive.mode != TINYGLTF_MODE_TRIANGLES  && primitive.mode != -1)
            throw std::runtime_error("Only TRIANGLES glTF primitives are supported");
        
        // Position

        const auto positionIterator = primitive.attributes.find("POSITION");
        if(positionIterator == primitive.attributes.end())  
            throw std::runtime_error( "glTF primitive has no POSITION");
        const std::vector<glm::vec3> positions = ReadVec3FloatAccessor(model,positionIterator->second);

        const std::size_t vertexCount = positions.size();
        if(vertexCount == 0) throw std::runtime_error("glTF Primitives has zero vertices");

        // Normal
        std::vector<glm::vec3> normals;
        const auto normalIterator = primitive.attributes.find("NORMAL");
        if(normalIterator != primitive.attributes.end()){
            normals = ReadVec3FloatAccessor(model, normalIterator->second);
            if(normals.size() != vertexCount) throw std::runtime_error("NORMAL count does not match POSITION count");
        }

        // TEXCOORD_0
        std::vector<glm::vec2> texCoords;
        const auto texCoordIterator = primitive.attributes.find("TEXCOORD_0");
        if(texCoordIterator != primitive.attributes.end()){
            texCoords = ReadVec2FloatAccessor(model,texCoordIterator->second);
            
            if(texCoords.size() != vertexCount) throw std::runtime_error( "TEXCOORD_0 count does not match POSITION count");
        }

        // TANGENT

        std::vector<glm::vec3> tangents;
        const auto tangentIterator = primitive.attributes.find("TANGENT");

        if (tangentIterator !=
            primitive.attributes.end())
        {
            tangents =ReadTangentAccessor(model,tangentIterator->second);

            if (tangents.size() != vertexCount) throw std::runtime_error("TANGENT count does not match POSITION count");
        }

        // Vertex Append

        /*
            기존 Primitive들이 이미 넣어놓은 Vertex 개수
            Primitive 내부 idnex는 0부터 시작, MeshData 전체 Index로 바꿀 때, 이 값을 더해야 함
        */
        const std::uint32_t baseVertex = static_cast<std::uint32_t>(result.vertices.size());
        result.vertices.reserve(
            result.vertices.size() + vertexCount
        );
        for(std::size_t i = 0; i < vertexCount; i++){
            Vertex vertex;
            vertex.position = positions[i];
            if(!normals.empty()) vertex.normal = normals[i];
            if(!texCoords.empty()) vertex.texCoord = texCoords[i];
            if(!tangents.empty()) vertex.tangent = tangents[i];

            result.vertices.push_back(vertex);
        }

        // Index Append
        const std::vector<std::uint32_t> localIndices = ReadIndices(model,primitive.indices,vertexCount);
        const std::uint32_t indexStart = static_cast<std::uint32_t>(result.indices.size());

        result.indices.reserve(result.indices.size() + localIndices.size());

        for(const std::uint32_t localIndex : localIndices){
            /*
                Primitive-local index 0, 1, 2 ..를 MeshData 전체 VertexIndex로 변환
                예 :
                    Primitive 0 Vertex = 100개
                    Primitive 1 index 0 -> meshData index 100
            */
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

// Node TransForm
/**
 * @brief 
 * 
 * @param source 
 * @param destination 
 */
void ReadNodeTransform(const tinygltf::Node& source, NodeData& destination){
    /*
        GLTF Node Transform의 두 방식
        1. Matrix
        2. TRS(Trans, Rot, Scale)
    */

    if(source.matrix.size() == 16){
        glm::mat4 matrix{1.0F};
        // glTF Matrix와 GLM은 모두 column-major convention 사용
        for(int column = 0; column < 4; column++){
            for(int row = 0; row <4; row++){
                matrix[column][row] = static_cast<float>(source.matrix[column*4 + row]);
            }
        }

        glm::vec3 scale{1.0F};
        glm::quat orientation{};
        glm::vec3 translation{0.0F};
        glm::vec3 skew{0.0F};
        glm::vec4 perspective{0.0F};
        if(!glm::decompose(matrix,scale,orientation,translation,skew,perspective)){
            throw std::runtime_error("Failed to decompose glTF node matrix");
        }

        destination.translation = translation;
        destination.rotation = glm::eulerAngles(glm::normalize(orientation));
        destination.scale = scale;
        return;
    }

    // Translation
    if(source.translation.size() == 3){
        destination.translation = glm::vec3{
            static_cast<float>(source.translation[0]),
            static_cast<float>(source.translation[1]),
            static_cast<float>(source.translation[2]),
        };
    }

    // Scale
    if(source.scale.size() == 3){
        destination.scale = glm::vec3{
            static_cast<float>(source.scale[0]),
            static_cast<float>(source.scale[1]),
            static_cast<float>(source.scale[2]),
        };
    }

    // Rotation
    if(source.rotation.size() == 4){
        /*
            glTF Quaternion : [x,y,z,w]
            GLM constructor : glm::quat(w,x,y,z)
        */

        const glm::quat quaternion{
            static_cast<float>(source.rotation[3]),
            static_cast<float>(source.rotation[0]),
            static_cast<float>(source.rotation[1]),
            static_cast<float>(source.rotation[2]),
        };

        /*
            현재 ECS Rotation이 Euler radian을 사용
            -> 쿼터니언 -> Euler 변환

            향후 Robot Joint 정확도를 더 엄격히 관리하려면
            Rotation을 Quaternion 기반으로 바꾼다.
        */
        destination.rotation = glm::eulerAngles(glm::normalize(quaternion));
    }
}
} // namespace 


// GLTFLoader

/**
 * @brief 
 * 
 * @param path 
 * @return ModelResource 
 */
ModelResource GltfLoader::LoadGLB(const std::filesystem::path& path){
    tinygltf::TinyGLTF loader;
    tinygltf::Model gltfModel;

    std::string error;
    std::string warning;

    const bool loaded = loader.LoadBinaryFromFile(&gltfModel,&error,&warning,path.string());
    if(!loaded) throw std::runtime_error("Failed to load GLM: " + path.string() + "\n" + error);

    /*
        warning은 로딩 실패가 아님
        Logger가 생기면 여기서 기록
        현재는 warning 무시
    */
    ModelResource result;

    // 파일 경로를 Prefix로 사용한다
    /*
        HCR12A_R00.glb#mesh/0
        HCR12A_R00.glb#mesh/1
        HCR12A_R00.glb#material/0
    */

    const std::string resourcePrefix = path.generic_string();
    // Texture
    result.textures.reserve(
        gltfModel.textures.size());

    for (std::size_t i = 0; i < gltfModel.textures.size(); ++i){
        result.textures.push_back(ConvertTexture( gltfModel, gltfModel.textures[i], resourcePrefix, i));
    }
    // Material
    result.materials.reserve(gltfModel.materials.size());
    for(std::size_t i = 0; i < gltfModel.materials.size(); i++){
        result.materials.push_back(ConvertMaterial(gltfModel.materials[i],resourcePrefix,i));
    }
    // Mesh
    result.meshes.reserve(gltfModel.meshes.size());

    for(std::size_t i = 0; i < gltfModel.meshes.size(); i++){
        result.meshes.push_back(ConvertMesh(gltfModel,gltfModel.meshes[i],resourcePrefix,i));
    }
    // Node
    result.nodes.resize(gltfModel.nodes.size());

    for(std::size_t i =0; i < gltfModel.nodes.size(); i++){
        const tinygltf::Node& source = gltfModel.nodes[i];
        NodeData& destination = result.nodes[i];

        // 이름 없는 노드도 내부에서 식별 가능하도록 이름 부여
        if(source.name.empty()) destination.name = "Node_" + std::to_string(i);
        else destination.name = source.name;

        destination.meshIndex = source.mesh;
        destination.childrenIndices = source.children;

        ReadNodeTransform(source,destination);
    }

    // Parent 복원
    /*
        glTF에는 children정보는 있지만, Parent 정보는 없다.
        ModelResource에서는 parentIndex도 필요함
        children 관계를 이용하여 역으로 복원한다.
    */
    for(std::size_t parentIndex = 0; parentIndex < result.nodes.size(); parentIndex++){
        const NodeData& parent = result.nodes[parentIndex];

        for(const int childIndex : parent.childrenIndices){
            if(childIndex < 0 || childIndex >= static_cast<int>(result.nodes.size()))
                throw std::runtime_error("Invalid child node index");
            
            NodeData& child = result.nodes[childIndex];

            // 일반 glTF Node는 하나의 Parent만 가짐
            if(child.parentIndex != -1) throw std::runtime_error("glTF node has multiple parents");
            child.parentIndex = static_cast<int>(parentIndex);
        }
    }

    // Root node
    int sceneIndex = gltfModel.defaultScene;
    // defaultScene이 지정되지 않았지만, Scene이 존재한다면 첫 번째 Scene 사용
    if(sceneIndex < 0 && !gltfModel.scenes.empty()) sceneIndex = 0;
    if(sceneIndex >= 0 && sceneIndex < static_cast<int>(gltfModel.scenes.size())){
        const tinygltf::Scene& scene = gltfModel.scenes[sceneIndex];

        /*
            현재 ModelResource는 rootNodeIndex 하나만 가진다.
            
            정규화된 나의 로봇팔 모델은 단일 Robot Root이므로 상관 X

            범용 glTF 지원하려면 향후:
                std::vector<int> rootNodeIndices; 로 바꾸면 됨
        */
        if(scene.nodes.size() == 1) result.rootNodeIndex = scene.nodes.front();
    }

    // Scene 정보로 Root를 못 찾은 경우
    // parentIndex == -1인 노드를 찾는다(Root가 정확히 하나일 때만 사용)
    if(result.rootNodeIndex < 0){
        int rootCandidate = -1;
        for(std::size_t i = 0; i < result.nodes.size(); i++){
            if(result.nodes[i].parentIndex != -1) continue;

            // 이미 다른 Root 후보가 있으면 multi-root 구조임
            if(rootCandidate != -1) {
                rootCandidate = -1; break;
            }

            rootCandidate = static_cast<int>(i);
        }
        result.rootNodeIndex = rootCandidate;
    }
    return result;
}
