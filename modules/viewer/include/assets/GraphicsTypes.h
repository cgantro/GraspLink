#pragma once

#include "assets/ResourceID.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

class Mesh;

/**
 * @brief GLB에서 읽은 정점 한 개의 CPU 속성이다.
 * @details
 * 좌표: position과 normal은 Mesh Local 기준이며 UV는 텍스처 좌표다.
 * 현재 지원: GltfLoader는 FLOAT 속성을 읽고 tangent는 VEC4의 xyz만 보존한다.
 * tangent의 w와 GPU vertex attribute 연결은 아직 없어 tangent 공간 기반 normal mapping에는 쓸 수 없다.
 */
struct Vertex
{
    /// Mesh Local 좌표의 정점 위치.
    glm::vec3 position{0.0f};
    /// Mesh Local 기준 정점 법선.
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
    /// 텍스처 이미지에서 색을 읽을 위치.
    glm::vec2 texCoord{0.0f};

    /// glTF VEC4 tangent의 xyz. w handedness와 GPU attribute 연결은 보존하지 않는다.
    glm::vec3 tangent{0.0F};
};

/**
 * @brief glTF Primitive 하나를 변환한 CPU 정점·index 데이터다.
 * @details
 * GltfLoader가 primitive별 index와 재질 번호를 읽고, Mesh 생성 시 여러 Primitive의 데이터를
 * 하나의 Mesh 배열과 SubMeshInfo 범위로 합친다. index 값은 byte 주소가 아닌 정점 배열의 원소 번호다.
 */
struct PrimitiveData
{
    std::vector<Vertex> vertices;
    std::vector<std::uint32_t> indices;

    /// ModelResource::materials 원소 번호. -1이면 기본 Material을 사용한다.
    int materialIndex = -1;
};

/**
 * @brief GLB 이미지에서 디코딩한 CPU 픽셀과 리소스 식별자다.
 * @details
 * GltfLoader는 현재 8-bit RGB/RGBA 이미지만 허용한다. AssetManager가 GPU Texture를 만들고
 * ResourceID 캐시에 보관하므로, 이 구조체의 pixels는 업로드 전 원본 데이터 역할을 한다.
 */
struct TextureData
{
    /// 모델 경로와 texture 번호로 만든 캐시 키.
    ResourceID uniqueID;

    /// 이미지 너비, 단위: pixel.
    int width = 0;
    /// 이미지 높이, 단위: pixel.
    int height = 0;
    /// 픽셀당 채널 수. 현재 3(RGB) 또는 4(RGBA).
    int channels = 0;

    /// 행 단위 패딩 없이 저장한 디코딩 픽셀 바이트.
    std::vector<unsigned char> pixels;
};

/**
 * @brief GLB 재질의 색상·PBR 값과 연결할 Texture ID다.
 * @details
 * GltfLoader가 glTF 값을 CPU에 보관하고 AssetManager가 runtime Material을 만든다.
 * 현재 AssetManager가 실제 Material에 연결하는 것은 baseColorTexture뿐이며 나머지 ID는
 * 로드되어도 렌더 재질 입력으로 사용되지 않는다.
 */
struct MaterialData
{
    /// 모델 경로와 material 번호로 만든 캐시 키.
    ResourceID uniqueID;
    /// GLB에 기록된 표시 이름. 비어 있을 수 있다.
    std::string name;

    /// 표면 기본 색과 알파 배율.
    glm::vec4 baseColorFactor{1.0f,1.0f,1.0f,1.0f};
    /// 금속성 계수, glTF 범위 0~1.
    float metallicFactor = 1.0f;
    /// 거칠기 계수, glTF 범위 0~1.
    float roughnessFactor = 1.0f;

    /// 표면에서 더해지는 발광 색.
    glm::vec3 emissiveFactor{0.0f};

    /// 기본 색 Texture의 캐시 ID. 픽셀과 GPU 객체는 AssetManager가 보유한다.
    ResourceID baseColorTexture;

    /// 금속성·거칠기 Texture의 캐시 ID. 현재 runtime Material에 연결되지 않는다.
    ResourceID metallicRoughnessTexture;

    /// 법선 Texture의 캐시 ID. 현재 runtime Material에 연결되지 않는다.
    ResourceID normalTexture;

    /// 차폐 Texture의 캐시 ID. 현재 runtime Material에 연결되지 않는다.
    ResourceID occlusionTexture;

    /// 발광 Texture의 캐시 ID. 현재 runtime Material에 연결되지 않는다.
    ResourceID emissiveTexture;
};

/**
 * @brief 합쳐진 Mesh index 배열에서 Primitive가 사용할 범위와 기본 재질이다.
 * @details
 * indexStart와 indexCount 단위는 uint32 index 원소다. Renderer가 draw 호출을 만들 때 시작 위치를
 * byte offset으로 변환한다. 재질 번호는 ModelResource::materials 기준이며 -1은 기본 Material이다.
 */
struct SubMeshInfo{
    /// MeshData::indices에서 시작하는 원소 위치.
    std::uint32_t indexStart = 0;
    /// 이 Primitive에 포함된 index 원소 수.
    std::uint32_t indexCount = 0;

    /// ModelResource::materials 원소 번호. -1이면 기본 Material을 사용한다.
    int defaultMaterialIndex = -1;
};

/**
 * @brief glTF Mesh의 CPU 배열, Primitive 범위와 업로드된 GPU Mesh 참조다.
 * @details
 * AssetManager가 vertices와 indices로 OpenGL Mesh를 만들고 gpuMesh에 공유 참조를 둔다.
 * PrefabFactory는 같은 참조를 MeshFilter에 복사하므로 Manager 캐시가 비워져도 Entity가 그리는 동안
 * GPU Mesh가 살아 있을 수 있다. 마지막 참조를 놓을 때 OpenGL Context가 유효해야 한다.
 */
struct MeshData
{
    /// 모델 경로와 mesh 번호로 만든 캐시 키.
    ResourceID uniqueID;

    /// 디버깅과 오류 메시지에 사용하는 Mesh 이름.
    std::string name;

    /// GPU 업로드 전 정점 속성 배열.
    std::vector<Vertex> vertices;

    /// 정점 배열을 참조하는 삼각형 index 배열.
    std::vector<std::uint32_t> indices;

    /// 원래 Primitive별 draw 범위와 기본 재질 번호.
    std::vector<SubMeshInfo> subMeshes;

    /// AssetManager가 업로드한 GPU Mesh. 업로드 전에는 비어 있다.
    std::shared_ptr<Mesh> gpuMesh;
};

/**
 * @brief glTF Node의 Local TRS와 Mesh 연결, 부모·자식 관계다.
 * @details
 * translation, rotation, scale은 모두 부모 Node 기준이다. translation은 glTF 장면의 길이 단위(m),
 * rotation은 단위 quaternion, scale은 배율이다. glTF 회전과 matrix 분해 결과를 같은 회전 표현으로 보관한다.
 * GltfLoader는 단일 root 아래의 트리만 허용하고, PrefabFactory가 이 관계를 Entity hierarchy로 옮긴다.
 * Mesh가 없는 Node도 관절 pivot과 자식 기준 변환을 유지하므로 삭제하지 않는다.
 */
struct NodeData
{
    /// 이름이 비어 있으면 GltfLoader가 Node 번호를 붙여 생성한다.
    std::string name;

    /// 부모 Node 기준 위치, 단위: m.
    glm::vec3 translation{
        0.0F
    };

    /// 부모 Node 기준 단위 quaternion. GLM 생성자 순서는 (w,x,y,z), 기본값은 항등 회전이다.
    glm::quat rotation{
        1.0F, 0.0F, 0.0F, 0.0F
    };

    /// 부모 Node 기준 크기 배율.
    glm::vec3 scale{
        1.0F
    };

    /// ModelResource::meshes 원소 번호. -1이면 Node 자체에 그릴 Mesh가 없다.
    int meshIndex = -1;

    /// ModelResource::nodes 부모 원소 번호. root는 -1.
    int parentIndex = -1;

    /// ModelResource::nodes에서 이 Node 바로 아래에 올 자식 원소 번호 목록.
    std::vector<int> childrenIndices;
};


/**
 * @brief GLB 한 파일에서 읽은 그래픽 데이터와 선택된 Scene root다.
 * @details
 * GltfLoader가 CPU 배열과 ID를 구성하고, AssetManager가 별도로 GPU 리소스를 업로드한다.
 * ModelResource는 GPU 객체를 직접 소유하지 않지만 각 MeshData의 shared_ptr은 GPU Mesh 수명에 참여한다.
 * 현재 loader는 선택한 Scene을 단일 root 트리로 제한하며 그 밖의 별도 Node 트리는 거부한다.
 */
struct ModelResource
{
    /// GLB 순서로 보관한 Material 입력값.
    std::vector<MaterialData> materials;

    /// GLB 순서로 보관한 Mesh 데이터.
    std::vector<MeshData> meshes;

    /// GLB Node와 hierarchy 데이터.
    std::vector<NodeData> nodes;

    /// GLB에서 디코딩한 이미지 데이터.
    std::vector<TextureData> textures;

    /// 선택한 Scene의 root Node 원소 번호. 찾지 못했으면 -1.
    int rootNodeIndex = -1;
};
