#pragma once

#include <cstddef>
#include <memory>

class Mesh;
class Shader;
class Material;

/**
 * @brief Entity가 공유할 GPU Mesh와 그릴 index 구간을 지정한다.
 * @details
 * RenderSystem이 Transform과 MeshRenderer를 함께 조회해 Renderer로 넘긴다. indexOffset은 index 원소 위치이고
 * Renderer가 uint32 index의 byte offset으로 바꾸므로 호출자는 byte 단위로 환산하지 않는다.
 * 여러 Primitive가 같은 Mesh를 가리킬 수 있다. mesh의 shared_ptr은 AssetManager 캐시와 독립된 소유 참조여서
 * Entity가 남아 있으면 캐시를 비워도 Mesh가 유지된다. 마지막 참조 해제는 OpenGL Context가 살아 있을 때 해야 한다.
 */
struct MeshFilter
{
    /// AssetManager가 올린 OpenGL Mesh의 공유 소유 참조. 비어 있으면 그릴 수 없다.
    std::shared_ptr<Mesh> mesh;

    /// Index Buffer에서 시작할 위치, 단위: uint32 index 원소.
    std::size_t indexOffset = 0U;

    /// 그릴 index 원소 수. 0이면 Mesh 전체 index 수를 사용하며, 이때 indexOffset은 0이어야 한다.
    std::size_t indexCount = 0U;
};

/**
 * @brief Entity의 Main pass 재질과 Shader, 렌더 포함 여부를 지정한다.
 * @details
 * 공유 참조는 AssetManager 캐시에서 얻은 객체의 수명을 연장한다. visible이 false이면 현재 Renderer의
 * Shadow pass와 Main pass 모두에서 제외된다. 이 Component는 draw 호출을 직접 하지 않고 MeshFilter,
 * Transform과 함께 RenderSystem 조회 결과로 사용된다.
 */
struct MeshRenderer
{
    /// Main pass에서 사용할 Shader의 공유 참조.
    std::shared_ptr<Shader> shader;

    /// 표면 색과 지원된 Texture를 제공하는 Material의 공유 참조.
    std::shared_ptr<Material> material;

    /// false이면 그림자와 화면 렌더링 대상에서 모두 제외한다.
    bool visible = true;
};
