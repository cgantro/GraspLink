#pragma once

#include <cstddef>
#include <memory>

class Mesh;
class Shader;
class Material;

/**
 * @brief RenderSystem이 그릴 Mesh와 Index Buffer 범위를 보관한다.
 *
 * @details indexOffset/indexCount를 사용하면 하나의 GPU Mesh 안에서 특정 SubMesh만
 * glDrawElements로 그릴 수 있다. indexCount가 0이면 전체 Mesh를 그리는 의미로 사용한다.
 */
/*
 * [추가 그래픽스 용어 설명]
 * - ECS Component: Entity에 붙이는 작은 데이터 조각. System은 필요한 Component 조합을 가진 Entity만 처리한다.
 * - MeshFilter: "어떤 Mesh의 어느 index 범위를 그릴지"를 지정한다.
 * - SubMesh: 하나의 Mesh 안에서 Material/Draw 범위가 다른 하위 구간.
 * - Index Offset: Index Buffer의 몇 번째 index부터 읽을지 나타내는 원소 단위 위치.
 * - Index Count: 그 draw에서 읽을 index 원소 개수. byte 수가 아니다.
 * - glDrawElements: Index Buffer를 이용해 Vertex를 재사용하며 triangle 등을 그리는 OpenGL draw 함수.
 */
struct MeshFilter
{
    // 실제 GPU Mesh를 여러 Entity가 공유할 수 있도록 shared_ptr로 참조한다.
    std::shared_ptr<Mesh> mesh;

    // Index Buffer에서 이 SubMesh가 시작되는 index 원소 위치. byte offset이 아니다.
    std::size_t indexOffset = 0U;

    // 그릴 index 원소 개수. 현재 contract에서 0은 전체 Mesh 범위를 사용한다는 특별한 의미다.
    std::size_t indexCount = 0U;
};

/**
 * @brief Mesh를 어떤 Shader/Material로 그릴지 지정하는 ECS Component.
 * @todo [FUTURE] render layer, cast/receive shadow, culling flag가 필요해지면 이 Component를 확장한다.
 */
/*
 * [추가 그래픽스 용어 설명]
 * - Shader: GPU가 Vertex 위치와 최종 pixel 색 등을 계산하는 프로그램.
 * - Material: 색/금속성/거칠기/Texture 등 표면 표현에 필요한 값 묶음.
 * - Visibility: Entity를 현재 rendering 대상에 포함할지 여부.
 */
struct MeshRenderer
{
    // 이 Mesh를 그릴 때 사용할 GPU Shader Program wrapper.
    std::shared_ptr<Shader> shader;

    // Shader에 전달할 표면 속성/Texture 묶음.
    std::shared_ptr<Material> material;

    // false이면 RenderSystem이 이 render item을 화면에 그리지 않도록 사용할 표시값.
    bool visible = true;
};
