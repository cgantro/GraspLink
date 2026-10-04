#pragma once

#include "assets/ResourceID.h"

#include <glm/glm.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

class Mesh;

/**
 * @file GraphicsTypes.h
 * @brief 파일 Loader와 GPU/ECS 계층 사이에서 사용하는 CPU-side 그래픽 중간 표현(IR).
 *
 * @details
 * 기본 데이터 흐름은 다음과 같다.
 *
 * `GLB -> TinyGLTF -> GltfLoader -> ModelResource -> AssetManager -> PrefabFactory -> Flecs Entity`
 *
 * 이 파일의 타입들은 glTF API 자체도 아니고 OpenGL/Flecs Component도 아니다.
 * Loader가 파일 포맷을 해석한 결과를 backend-neutral하게 보존하여 파일 파싱과 GPU/ECS 생성을 분리한다.
 *
 * 좌표/단위 규칙:
 * - GltfLoader는 position/translation 수치를 임의로 scale하지 않고 asset의 numeric value를 그대로 복사한다.
 * - 현재 controller-ready HCR-12A + 2F-85 asset은 meter 단위로 정규화되어 있으므로 해당 모델의 position/translation은 [m]다.
 * - rotation은 현재 ECS 저장 형식 때문에 Euler radians [rad]로 보존한다.
 * - scale, normal, tangent, UV, PBR factor는 무차원 값이다.
 */

/**
 * @brief 하나의 렌더링 정점에 필요한 vertex attribute 묶음.
 *
 * @details
 * Mesh가 GPU에 업로드할 때 현재 attribute layout은 다음과 같다.
 * - location 0: position vec3
 * - location 1: normal vec3
 * - location 2: texCoord vec2
 *
 * tangent는 CPU-side에서 보존하지만 현재 shader attribute로 연결되지 않는다.
 */
struct Vertex
{
    /**
     * @brief Mesh local frame의 정점 위치.
     * @note Loader가 단위 변환을 하지 않으므로 asset 단위를 그대로 따른다. HCR controller-ready asset은 [m].
     */
    glm::vec3 position{0.0F};

    /**
     * @brief 조명 계산용 vertex normal 방향벡터.
     * @note 무차원이며 일반적으로 unit length를 기대한다. 기본값은 +Y.
     */
    glm::vec3 normal{0.0F, 1.0F, 0.0F};

    /** @brief 2D texture sampling용 UV 좌표. 무차원. */
    glm::vec2 texCoord{0.0F};

    /**
     * @brief Normal mapping용 tangent xyz 방향벡터.
     *
     * @warning glTF TANGENT는 원래 vec4이며 w에 bitangent handedness(+1/-1)가 들어 있다.
     *          현재 Loader는 xyz만 보존하므로 정확한 normal mapping에 바로 사용하면 안 된다.
     * @todo [FUTURE] normal mapping 도입 시 vec4로 확장해 handedness를 보존한다.
     */
    glm::vec3 tangent{0.0F};
};

/**
 * @brief 하나의 glTF Primitive를 읽는 중간 단계에서 사용하는 CPU geometry + material 참조.
 *
 * @details
 * glTF Mesh 하나가 여러 Primitive를 가질 수 있고 Primitive 하나는 하나의 Material을 참조한다.
 * GltfLoader는 이후 여러 Primitive를 MeshData의 단일 vertex/index 배열로 합치면서 SubMeshInfo를 만든다.
 */
struct PrimitiveData
{
    /** @brief Primitive 전용 vertex 배열. */
    std::vector<Vertex> vertices;

    /** @brief vertices 배열을 참조하는 local uint32 index 배열. */
    std::vector<std::uint32_t> indices;

    /** @brief ModelResource::materials index. -1이면 Material 미지정. */
    int materialIndex = -1;
};

/**
 * @brief TinyGLTF가 decode한 image를 GPU upload 전까지 보관하는 CPU-side texture 데이터.
 */
struct TextureData
{
    /** @brief AssetManager cache에서 사용하는 stable resource key. */
    ResourceID uniqueID;

    /** @brief Image 가로 pixel 수. */
    int width = 0;

    /** @brief Image 세로 pixel 수. */
    int height = 0;

    /** @brief Pixel당 channel 수. 현재 Loader는 RGB(3) 또는 RGBA(4)를 지원한다. */
    int channels = 0;

    /** @brief 8-bit decoded pixel byte 배열. row/channel layout은 TinyGLTF가 제공한 image를 그대로 보존한다. */
    std::vector<unsigned char> pixels;
};

/**
 * @brief glTF PBR Metallic-Roughness material의 CPU-side 표현.
 *
 * @details
 * OpenGL Texture/Shader 객체를 직접 소유하지 않는다. Texture는 ResourceID만 보관하고
 * AssetManager가 GPU Texture를 생성/캐시한다.
 */
struct MaterialData
{
    /** @brief Material cache key. 보통 `파일경로#material/index`를 hash해 생성한다. */
    ResourceID uniqueID;

    /** @brief glTF Material 표시 이름. */
    std::string name;

    /**
     * @brief PBR base color RGBA factor. 각 성분은 일반적으로 0..1 범위의 무차원 계수.
     * @note BaseColorTexture가 있으면 shader에서 texture color와 곱해 사용한다.
     */
    glm::vec4 baseColorFactor{1.0F, 1.0F, 1.0F, 1.0F};

    /** @brief PBR metallic factor. 0=dielectric, 1=metallic에 해당하는 무차원 계수. */
    float metallicFactor = 1.0F;

    /** @brief PBR roughness factor. 0=smooth, 1=rough에 해당하는 무차원 계수. */
    float roughnessFactor = 1.0F;

    /** @brief Emissive RGB factor. 최종 발광색은 emissive texture와 조합될 수 있다. */
    glm::vec3 emissiveFactor{0.0F};

    /** @brief Base-color texture ResourceID. invalid(0)이면 texture 없음. */
    ResourceID baseColorTexture;

    /** @brief Metallic-roughness texture ResourceID. invalid(0)이면 texture 없음. */
    ResourceID metallicRoughnessTexture;

    /** @brief Normal-map texture ResourceID. invalid(0)이면 texture 없음. */
    ResourceID normalTexture;

    /** @brief Occlusion texture ResourceID. invalid(0)이면 texture 없음. */
    ResourceID occlusionTexture;

    /** @brief Emissive texture ResourceID. invalid(0)이면 texture 없음. */
    ResourceID emissiveTexture;
};

/**
 * @brief 하나의 GPU Mesh index buffer 안에서 특정 Primitive가 사용하는 draw 범위.
 *
 * @details
 * `indexStart`와 `indexCount`는 index "byte offset"이 아니라 index 원소 기준 offset/count다.
 * Renderer에서 실제 glDrawElements byte offset으로 바꿀 때 `indexStart * sizeof(uint32_t)`가 필요하다.
 */
struct SubMeshInfo
{
    /** @brief 전체 Mesh index 배열에서 이 SubMesh가 시작하는 index 원소 번호. */
    std::uint32_t indexStart = 0;

    /** @brief 이 SubMesh가 그릴 index 원소 개수. Triangle primitive라면 일반적으로 3의 배수. */
    std::uint32_t indexCount = 0;

    /** @brief ModelResource::materials index. -1이면 fallback/default material 사용. */
    int defaultMaterialIndex = -1;
};

/**
 * @brief 하나의 glTF Mesh를 표현하는 CPU-side geometry asset과 GPU upload 결과.
 *
 * @details
 * GltfLoader가 모든 Primitive의 vertex/index를 합쳐 CPU 배열과 SubMeshInfo를 만들고,
 * AssetManager가 이를 실제 GPU Mesh(VAO/VBO/EBO)로 업로드해 gpuMesh를 채운다.
 */
struct MeshData
{
    /** @brief Mesh cache key. */
    ResourceID uniqueID;

    /** @brief glTF Mesh 표시 이름. */
    std::string name;

    /** @brief 합쳐진 vertex 배열. position 단위는 source asset을 따른다. */
    std::vector<Vertex> vertices;

    /** @brief 합쳐진 uint32 index 배열. */
    std::vector<std::uint32_t> indices;

    /** @brief Primitive별 draw range/material mapping. */
    std::vector<SubMeshInfo> subMeshes;

    /**
     * @brief AssetManager가 생성한 GPU-side Mesh shared ownership.
     * @note Load 직후에는 nullptr이고 UploadModel() 이후 채워진다.
     */
    std::shared_ptr<Mesh> gpuMesh;
};

/**
 * @brief GLB/glTF Node hierarchy의 한 노드를 vector-index 방식으로 표현한다.
 *
 * @details
 * Pointer tree 대신 ModelResource::nodes index를 사용해 lifetime/복사/직렬화 문제를 단순화한다.
 * PrefabFactory가 이 데이터를 `(Position,Local)/(Rotation,Local)/(Scale,Local)` Component로 변환한다.
 */
struct NodeData
{
    /** @brief glTF Node 이름. Robot asset에서는 J1~J6 같은 semantic key로도 사용된다. */
    std::string name;

    /**
     * @brief Parent Node 기준 local translation.
     * @note Loader는 scale 변환 없이 값을 복사한다. controller-ready HCR asset에서는 [m].
     */
    glm::vec3 translation{0.0F};

    /**
     * @brief Parent Node 기준 local Euler rotation [rad].
     *
     * glTF rotation quaternion [x,y,z,w] 또는 node matrix를 GltfLoader에서 quaternion으로 복원한 뒤
     * 현재 ECS Rotation(vec3)에 맞추기 위해 Euler radians로 변환한다.
     */
    glm::vec3 rotation{0.0F};

    /** @brief Parent Node 기준 local scale. 무차원, 기본값 (1,1,1). */
    glm::vec3 scale{1.0F};

    /**
     * @brief ModelResource::meshes index.
     * @note -1이면 transform-only Node. Robot Joint pivot처럼 Mesh가 없는 노드를 보존하는 데 중요하다.
     */
    int meshIndex = -1;

    /** @brief ModelResource::nodes에서 부모 index. -1이면 root node. */
    int parentIndex = -1;

    /** @brief ModelResource::nodes에서 직접 자식 Node들의 index 배열. */
    std::vector<int> childrenIndices;
};

/**
 * @brief 하나의 GLB/Model 파일 전체를 표현하는 CPU-side asset resource.
 *
 * @details
 * 이 구조만으로 AssetManager가 GPU 자원을 만들고 PrefabFactory가 Flecs Entity hierarchy를 복원할 수 있어야 한다.
 */
struct ModelResource
{
    /** @brief 파일에 포함된 PBR material 배열. */
    std::vector<MaterialData> materials;

    /** @brief 파일에 포함된 mesh 배열. */
    std::vector<MeshData> meshes;

    /** @brief GLB hierarchy 전체 Node 배열. parent/children index로 tree를 복원한다. */
    std::vector<NodeData> nodes;

    /** @brief 파일에 포함된 decoded texture image 배열. */
    std::vector<TextureData> textures;

    /**
     * @brief 단일-root 모델용 대표 root node index. -1이면 유효한 root가 정해지지 않음.
     * @warning glTF Scene은 여러 root node를 가질 수 있으므로 multi-root 지원이 필요해지면 vector로 확장해야 한다.
     */
    int rootNodeIndex = -1;
};
