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
 * @brief 모델 표면의 꼭짓점 한 개와 그릴 때 필요한 표면 정보를 담는다.
 * @details
 * Mesh는 여러 삼각형의 꼭짓점과 연결 번호를 모은 모델 형상이다. position은 Mesh 자체 기준(Local) 위치고 normal(법선)은 그 꼭짓점 표면에 수직인 방향이다.
 * texCoord(UV)는 표면에서 색상 Texture의 어느 픽셀을 읽을지 나타내는 이미지 좌표다. GltfLoader는 실수형 위치·법선·UV만 읽는다.
 * tangent는 표면을 따라가는 방향이고 normal(법선)은 표면에 수직인 방향이다. 원본 VEC4의 w 부호는 tangent와 법선에서 두 방향과 수직인 나머지 표면 방향을 정한다.
 * 현재 w는 버리고 tangent도 GPU에 전달하지 않는다. Normal mapping은 이미지의 방향 정보를 이용해 표면을 울퉁불퉁하게 보이게 하는 방법이며 현재 사용할 수 없다.
 */
struct Vertex
{
    /// Mesh 기준 좌표의 위치. 모델 변환이 적용되기 전 값이다.
    glm::vec3 position{0.0f};
    /// Mesh 기준 표면의 수직 방향.
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
    /// 색상 이미지 안에서 읽을 위치. 각 성분은 이미지 크기에 대한 비율이다.
    glm::vec2 texCoord{0.0f};

    /// tangent는 표면을 따라가는 방향이다. 원본 네 성분 중 xyz만 보존하며 w의 부호는 tangent와 법선에서 나머지 표면 방향을 정하는 데 쓰이지만 현재 버린다.
    glm::vec3 tangent{0.0F};
};

/**
 * @brief 모델 표면을 그리는 한 덩어리의 꼭짓점과 삼각형 연결 정보를 담는다.
 * @details
 * glTF Primitive는 하나의 재질로 그릴 삼각형 꼭짓점 묶음이다. GltfLoader가 묶음별 꼭짓점과 삼각형 연결 번호를 읽고 Mesh를 만들 때 배열 뒤에 이어 붙인다.
 * index는 삼각형 세 꼭짓점을 고르는 vertices 배열 번호다. byte 위치와 달리 0은 첫 꼭짓점, 1은 두 번째 꼭짓점을 뜻한다.
 */
struct PrimitiveData
{
    std::vector<Vertex> vertices;
    std::vector<std::uint32_t> indices;

    /// 사용할 재질의 ModelResource::materials 번호. -1이면 기본 재질을 사용한다.
    int materialIndex = -1;
};

/**
 * @brief GLB 이미지 파일을 픽셀 바이트로 풀어 둔 CPU 데이터다.
 * @details
 * GltfLoader는 색의 빨강·초록·파랑 및 선택적 alpha 채널마다 8 bit를 쓰는 RGB 또는 RGBA 이미지로 읽는다. AssetManager가 이 바이트를 GPU 이미지로 복사하고 캐시에 보관한다. 따라서 pixels는 GPU 업로드 전까지 사용하는 원본이다.
 */
struct TextureData
{
    /// 모델 파일 경로와 glTF texture 항목 번호를 조합한 식별자.
    ResourceID uniqueID;

    /// 이미지 너비, 단위: pixel.
    int width = 0;
    /// 이미지 높이, 단위: pixel.
    int height = 0;
    /// 픽셀당 채널 수. 현재 3(RGB) 또는 4(RGBA).
    int channels = 0;

    /// 위에서 아래 순서의 픽셀 바이트. 행 사이에 빈 패딩이 없다.
    std::vector<unsigned char> pixels;
};

/**
 * @brief 표면 색·반사 계수와 기본색 이미지 연결 정보를 담는다.
 * @details
 * glTF 재질은 표면 색과 빛 반응을 정하는 값의 묶음이다. GltfLoader는 기본색, 금속성·거칠기 계수와 발광색 숫자를 읽고 기본색 이미지 참조만 기록한다.
 * 다른 이미지 참조 필드는 빈 식별자로 남아 화면용 재질에 연결되지 않는다. 모델 이미지 배열을 읽어 GPU에 올리는 작업과 재질이 그 이미지를 사용하도록 연결하는 작업은 별개다.
 * 금속성·거칠기 숫자는 화면용 재질에 전달되지만 발광색 숫자는 저장만 하고 현재 화면 계산에는 전달하지 않는다.
 */
struct MaterialData
{
    /// 모델 파일 경로와 재질 번호를 조합한 식별자.
    ResourceID uniqueID;
    /// GLB에 기록된 표시 이름. 비어 있을 수 있다.
    std::string name;

    /// 표면 기본 RGB 색과 투명도에 곱할 값.
    glm::vec4 baseColorFactor{1.0f,1.0f,1.0f,1.0f};
    /// 금속성 계수, glTF 범위 0~1.
    float metallicFactor = 1.0f;
    /// 거칠기 계수, glTF 범위 0~1.
    float roughnessFactor = 1.0f;

    /// 파일에서 읽은 발광 색. 현재 화면용 재질과 shader에는 전달하지 않는다.
    glm::vec3 emissiveFactor{0.0f};

    /// 기본색 이미지 연결 정보. 해당 픽셀과 GPU 객체는 AssetManager가 관리한다.
    ResourceID baseColorTexture;

    /// 금속성·거칠기 이미지 참조. 현재는 빈 식별자로 남고 화면용 재질에 연결되지 않는다.
    ResourceID metallicRoughnessTexture;

    /// 표면 기울기 이미지 참조. 현재는 빈 식별자로 남고 화면용 재질에 연결되지 않는다.
    ResourceID normalTexture;

    /// 빛을 가리는 정도를 담는 이미지 참조. 현재는 빈 식별자로 남고 화면용 재질에 연결되지 않는다.
    ResourceID occlusionTexture;

    /// 표면이 내는 빛을 담는 이미지 참조. 현재는 빈 식별자로 남고 화면용 재질에 연결되지 않는다.
    ResourceID emissiveTexture;
};

/**
 * @brief 이어 붙인 꼭짓점 연결 배열 중 한 표면 묶음의 범위와 재질을 가리킨다.
 * @details
 * indexStart는 첫 연결 번호의 위치이고 indexCount는 사용할 연결 번호 개수다. 둘 다 4-byte 정수 원소 단위다. Renderer는 GPU 호출 전에 시작 위치를 byte 단위로 바꾼다. 재질 번호 -1은 기본 재질을 뜻한다.
 */
struct SubMeshInfo{
    /// MeshData::indices 배열에서 이 묶음의 첫 연결 번호 위치.
    std::uint32_t indexStart = 0;
    /// 이 묶음이 사용할 꼭짓점 연결 번호의 개수.
    std::uint32_t indexCount = 0;

    /// ModelResource::materials에서 선택할 재질 번호. -1이면 기본 재질을 사용한다.
    int defaultMaterialIndex = -1;
};

/**
 * @brief 한 모델 Mesh의 CPU 데이터, 표면별 범위, GPU에 올린 Mesh 참조를 모은다.
 * @details
 * AssetManager가 vertices와 indices를 그래픽 카드 메모리에 올려 gpuMesh를 만든다.
 * PrefabFactory는 gpuMesh 참조를 Entity의 MeshFilter에도 복사한다. 그래서 캐시를 비운 뒤에도 Entity가 그리는 동안 GPU 데이터가 유지된다. 마지막 참조를 해제할 때 OpenGL context가 현재 스레드에서 활성화되어 있어야 한다.
 */
struct MeshData
{
    /// 모델 파일 경로와 형상 번호를 조합한 식별자.
    ResourceID uniqueID;

    /// 디버깅과 오류 메시지에 사용하는 Mesh 이름.
    std::string name;

    /// GPU 업로드 전 정점 속성 배열.
    std::vector<Vertex> vertices;

    /// 삼각형을 만들 꼭짓점 번호의 연속 배열. 번호는 vertices의 위치를 가리킨다.
    std::vector<std::uint32_t> indices;

    /// 원래 Primitive별 draw 범위와 기본 재질 번호.
    std::vector<SubMeshInfo> subMeshes;

    /// AssetManager가 업로드한 GPU Mesh. 업로드 전에는 비어 있다.
    std::shared_ptr<Mesh> gpuMesh;
};

/**
 * @brief 모델 안 한 부품의 위치·회전·크기와 부모·자식, 그릴 Mesh 번호를 저장한다.
 * @details
 * 위치(translation), 회전(rotation), 크기(scale)는 부모 부품 기준 값이다. 위치 단위는 m이고 크기는 배율이다.
 * Quaternion은 회전축과 회전량을 네 숫자로 나타내는 방식이며 yaw/pitch/roll처럼 세 축 각도를 차례로 적용하는 방식과 다르다.
 * 이 회전은 길이가 1인 quaternion으로 저장한다. 파일이 회전을 행렬로 적었든 네 숫자로 적었든 같은 quaternion 형태로 저장한다.
 * GltfLoader는 부모가 하나인 나무 모양 관계만 허용하고 PrefabFactory가 이를 Entity 관계로 옮긴다.
 * Mesh가 없는 부품도 관절 중심점이나 자식 위치의 기준이 될 수 있어 보존한다.
 */
struct NodeData
{
    /// 이름이 비어 있으면 GltfLoader가 Node 번호를 붙여 생성한다.
    std::string name;

    /// 부모 부품 기준 위치 [m].
    glm::vec3 translation{
        0.0F
    };

    /// 부모 부품 기준 회전. 길이가 1인 quaternion이며 GLM 생성자 인자 순서는 (w,x,y,z)다.
    glm::quat rotation{
        1.0F, 0.0F, 0.0F, 0.0F
    };

    /// 부모 부품 기준 크기 배율. (1,1,1)은 원래 크기다.
    glm::vec3 scale{
        1.0F
    };

    /// ModelResource::meshes에서 그릴 형상의 번호. -1이면 이 부품에는 형상이 없다.
    int meshIndex = -1;

    /// ModelResource::nodes에서 부모 부품의 번호. 최상위 부품은 -1이다.
    int parentIndex = -1;

    /// 이 부품 바로 아래에 연결할 자식 번호 목록.
    std::vector<int> childrenIndices;
};


/**
 * @brief GLB 파일에서 읽은 이미지·재질·형상·부품 관계를 한데 모은 결과다.
 * @details
 * GltfLoader는 우선 CPU 배열을 만든다. AssetManager가 이를 그래픽 카드에 올린다.
 * 각 MeshData의 공유 참조는 GPU Mesh가 언제 해제될지 결정하는 데 참여한다.
 * 현재는 선택한 장면이 최상위 부품 하나에서 시작하는 계층만 허용하며, 장면 밖의 별도 부품 묶음은 거부한다.
 */
struct ModelResource
{
    /// 파일에 나온 순서대로 저장한 재질 입력값.
    std::vector<MaterialData> materials;

    /// 파일에 나온 순서대로 저장한 형상 데이터.
    std::vector<MeshData> meshes;

    /// 부품의 위치·부모·자식 관계 데이터.
    std::vector<NodeData> nodes;

    /// 파일에서 픽셀로 풀어 둔 이미지 데이터.
    std::vector<TextureData> textures;

    /// 사용할 장면의 최상위 부품 번호. 없으면 -1이다.
    int rootNodeIndex = -1;
};
