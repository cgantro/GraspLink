#pragma once

#include "assets/ResourceID.h"

#include <glm/glm.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

class Mesh;

/* 
    GltfLoader가 GLB/glTF를 읽은 결과를 저장하는 중간 데이터 구조

     중요한 데이터 흐름:
        GLB
         ↓
      tinygltf
         ↓
      GltfLoader
         ↓
      ModelResource
         ↓
      AssetManager
         ↓
      PrefabFactory
         ↓
      Flecs Entity
    
    이 파일의 타입들:
        glTF, flecs 컴포넌트, OpenGL API도 아님

    파일 포맷과 ECS 사이에 존재하는
    IR(Intermediate Representation)
        GltfLoader와 ECS를 직접 결합하지 않기 위해
    
    추후 OBJ, 자체 포맷등의 Loader가 추가되어도
        ModelResource만 만들면 재사용 가능
*/

/*
    Vertex
        pos, norm, texCoord
        위치 : X, Y, Z
        법선 벡터 : 주로 크기 1로 정규화된 X,Y,Z 
            빛 연산의 핵심
        texCoord : 텍스처 좌표 U/V
            3D모델 표면에 2D 텍스처를 입힐 때 사용하는 2차원 좌표
*/
struct Vertex
{
    // Local Mesh 좌표계에서의 정점 위치.
    glm::vec3 position{0.0f};
    // 조명 계산에 사용하는 정점 법선. 기본값은 +Y 방향이다.
    glm::vec3 normal{0.0f, 1.0f, 0.0f};
    // 텍스처의 U/V 좌표.
    glm::vec2 texCoord{0.0f};

    /*  

        Normal Mapping(물체의 표면을 입체적이고 사실적으로 표현)에서 탄젠트 공간을 만들기 위함
            메모리를 최적화하고 정확한 방향 계산

        탄젠트 공간(T(접선),B(종법선),N(법선) 서로 수직인 세 개의 축)
            - 각 정점마다 주어지는 정점 기준의 로컬 3차원 좌표계
        
            - 이미 정점에 Normal 있음
            - 아래처럼 Tangent(T)를 불러 왔다면
            - BiTangent는 외적을 통해 구할 수 있음
            - float3 크기의 Bitangent 데이터를 담아서 GPU로 보내는 것은 낭비임
        Bitangent는 별도 저장 X
            Shader에서 B = cross(N,T) 형식으로 만들 수 있음
        
        glTF tangent의 실제 값은 vec4이며 w에 handedness
            - 좌표계의 방향성(왼손/오른손 법칙)
            - uv좌표가 대칭이거나 뒤집혀 있는 경우
                - B = cross(N,T)시에, B가 실제 텍스처와 반대로 뒤집히는 경우 발생
                - 이를 보정하기 위해 glTF 표준 포맷 -> 탄젠트 값을 vec4로 제공
            - w : 1/-1(좌표계 뒤집힙 여부)
        Normal Mapping을 붙일 때는 이 구조를 vec4로 확장
    */
    glm::vec3 tangent{0.0F};
};

/*
    PrimitiveData
    
    1. Primitive
        Mesh를 이루는 가장 최소 단위의 Draw Call 메쉬 덩어리
        - Mesh는 여러 Prim으로 쪼개져 있을 수 있음
        - Prim은 단 하나의 Material만 가짐
        - 정점의 pos, norm, tangent등의 버퍼 데이터를 직접 참조한다.
    2. Material
        물체의 표면이 빛과 어떻게 상호작용하는지(색상, 거칠기, 금속성 등)을 정의하는 데이터셋
        텍스처는 이미지, 메테리얼은 이미지를 포함하여 물리적 질감을 표현하는 수학 설정
        - 노멀 맵 + 베이스 컬러 맵, 거칠기 맵 등의 텍스처 이미지 + 셰이더 설정 값
        - Prim마다 materialID가 할당되어 있음
            - GPU가 prim을 그릴때 이 텍스처와 노멀맵을 바인딩 가능
    3. Index Range
        거대한 하나의 Index Buffer안에서 특정 Prim이 사용하는 시작 위치와 개수
        - Offset, Count
        - 메모리 효율을 위해, 엔진들은 모든 Prim의 정점과 인덱스 데이터를 하나의 통버퍼에 저장
            - VBO EBO
        - 이때 Index buffer의 특정 위치(offset)부터 cnt만큼 읽어서 tri그려라 하는 역할을 함
    glTF :
        Mesh
            - Prim 0
            - Prim 1
            - Prim 2
    하나의 Prim은 보통 하나의 Material과 index Range를 가짐

    GltfLoader가 각 Primitive를 읽는 중간 단계에서 사용한다.
*/
struct PrimitiveData
{
    std::vector<Vertex> vertices;
    std::vector<std::uint32_t> indices;

    /*
        ModelResource::materials의 index
        -1 -> Material이 지정되지 않은 Prim
    */
   int materialIndex = -1;
};

struct TextureData
{
    ResourceID uniqueID;

    int width = 0;
    int height = 0;
    int channels = 0;

    std::vector<unsigned char> pixels;
};

/*
    glTF PBR Metallic-Roughness Material를
    내부 표현으로 변환한 구조

    이 구조체 자체는 OpenGL Texture를 소유하지 않는다.

    Texture는 AssetManager가 관리한다.
    여기선 ResourceID만 보관
*/

struct MaterialData
{
    ResourceID uniqueID;
    std::string name;

    // PBR Factors
    // 물리기반 렌더링 요소

    // 알베도, 조명/그림자가 배제된 물체 본연의 순수 색
    glm::vec4 baseColorFactor{1.0f,1.0f,1.0f,1.0f};
    // 물체의 금속성/비금속성 구분 -> 빛 반사율 조정
    float metallicFactor = 1.0f;
    // 표면의 거친 정도(빛의 퍼짐/뚜렷함 반사 결정)
    float roughnessFactor = 1.0f;

    // 물체 표면이 스스로 빛을 내는 강도와 색상을 조절하는 RGB 계수
    // 최종 발광 색상 = emissiveTexture X emissiveFactor
    // emissiveTexture
    //      존재시 -> 텍스처 이미지의 각 픽셀 색상에 Factor 곱
    //      없음 -> factor 값이 물체 표면 전체의 단색 발광색이 됨 
    glm::vec3 emissiveFactor{0.0f};

    // ------------------------------------------------------------------------
    // PBR Texture Resources
    // ------------------------------------------------------------------------

    /*
        값이 0인 ResourceID는 해당 Texture가 없다는 의미다.
    */

    ResourceID baseColorTexture;

    ResourceID metallicRoughnessTexture;

    ResourceID normalTexture;

    ResourceID occlusionTexture;

    ResourceID emissiveTexture;
};

/*
    하나의 GPU Mesh 안에서 특정 Primitive가 사용하는
    Index Buffer 범위를 표현한다.

    indices:
        [------------ 전체 Mesh ------------]
        | primitive 0 | primitive 1 | primitive 2 |
    primitive 1을 그릴 때:
        indexStart = primitive 1 시작 index
        indexCount = primitive 1 index 개수

    glDrawElements의 마지막 offset으로 연결됨
*/
struct SubMeshInfo{
    std::uint32_t indexStart = 0;
    std::uint32_t indexCount = 0;

    // 이 SubMesh가 기본적으로 사용할 Material.
    // ModelResource::materials의 idx
    int defaultMaterialIndex = -1;
};

// ============================================================================
// MeshData
// ============================================================================

/*
    CPU에 로드된 하나의 Mesh Asset.

    GltfLoader는 CPU vertex/index 데이터를 만든다.

        GltfLoader
            ↓
        vertices / indices

    이후 AssetManager가 GPU Mesh를 생성한다.

        vertices / indices
            ↓
        Mesh
            ↓
        VBO / IBO / VAO


    GltfLoader가 직접 OpenGL 객체를 만들지 않는 것이 중요하다.

        GltfLoader
            = 파일 해석

        AssetManager / Graphics
            = GPU Resource 생성

    책임을 분리하기 위함이다.
*/
struct MeshData
{
    ResourceID uniqueID;

    std::string name;

    std::vector<Vertex> vertices;

    std::vector<std::uint32_t> indices;

    /*
        하나의 glTF Mesh가 여러 Primitive를 가질 수 있으므로
        각 Draw Range를 별도로 저장한다.
    */
    std::vector<SubMeshInfo> subMeshes;


    /*
        GPU에 업로드된 최종 Mesh.

        AssetManager가 생성한 뒤 채운다.

        shared_ptr인 이유:

        AssetManager와 여러 Entity가 같은 Mesh Resource를
        공유할 수 있기 때문이다.

        ResourceID를 가진 ECS Component는 직접 이 shared_ptr을
        들고 있지 않고 AssetManager를 통해 접근한다.
    */
    std::shared_ptr<Mesh> gpuMesh;
};


// ============================================================================
// NodeData
// ============================================================================

/*
    GLB의 Node hierarchy를 표현한다.

    예:

        Robot
        └─ Base
           └─ J1
              └─ Link1
                 └─ J2


    ModelResource에서는 pointer tree로 만들지 않고
    vector + index 구조를 사용한다.


    이유:

    1. serialization이 쉽다.
    2. pointer lifetime 문제가 없다.
    3. ModelResource 이동/복사 시 구조가 덜 깨진다.
    4. PrefabFactory에서 index를 이용해 hierarchy를 복원하기 쉽다.
*/
struct NodeData
{
    std::string name;


    // ------------------------------------------------------------------------
    // Local Transform
    // ------------------------------------------------------------------------

    /*
        모두 Parent Node 기준 Local Transform이다.

        이후 PrefabFactory에서:

            (Position, Local)
            (Rotation, Local)
            (Scale, Local)

        Component로 변환된다.
    */

    glm::vec3 translation{
        0.0F
    };

    /*
        현재 ECS Rotation이 Euler vec3를 사용하므로
        NodeData도 Euler radians로 맞춘다.

        GltfLoader에서 glTF Quaternion을 읽은 뒤
        Euler로 변환한다.
    */
    glm::vec3 rotation{
        0.0F
    };

    glm::vec3 scale{
        1.0F
    };


    // ------------------------------------------------------------------------
    // Resource / Hierarchy
    // ------------------------------------------------------------------------

    /*
        ModelResource::meshes의 index.

        Mesh가 없는 transform-only Node라면 -1.

        로봇 모델에서는 Joint pivot 역할을 하는 Node가
        Mesh 없이 존재할 수도 있으므로 이 경우가 중요하다.
    */
    int meshIndex = -1;


    /*
        ModelResource::nodes의 부모 index.

        Root Node는 -1.
    */
    int parentIndex = -1;


    /*
        ModelResource::nodes에서 Child Node들의 index.
    */
    std::vector<int> childrenIndices;
};


// ============================================================================
// ModelResource
// ============================================================================

/*
    하나의 GLB/Model 파일 전체를 나타내는 CPU-side Resource.

        HCR12A_R00.glb
            ↓
        ModelResource
        ├─ Materials
        ├─ Meshes
        └─ Nodes


    이 데이터만 가지고 PrefabFactory가
    Flecs Entity hierarchy를 만들 수 있어야 한다.
*/
struct ModelResource
{
    std::vector<MaterialData> materials;

    std::vector<MeshData> meshes;

    std::vector<NodeData> nodes;

    std::vector<TextureData> textures;

    /*
        단일 Root 모델을 위한 편의 값.

        단, glTF Scene은 여러 root node를 가질 수 있다.

        따라서 GltfLoader 구현 시 이 값 하나만 믿고
        multi-root GLB를 버리는 구조로 만들면 안 된다.

        필요해지면:

            std::vector<int> rootNodeIndices;

        로 확장한다.
    */
    int rootNodeIndex = -1;
};
