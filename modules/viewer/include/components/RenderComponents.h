#pragma once

#include <cstddef>
#include <memory>

class Mesh;
class Shader;
class Material;

/**
 * @brief RenderSystem이 그릴 GPU Mesh와 Index Buffer 범위를 연결하는 ECS Component.
 *
 * @details
 * 하나의 GPU Mesh가 여러 glTF Primitive/SubMesh를 합쳐 보관할 수 있으므로
 * `indexOffset/indexCount`로 그중 특정 범위만 선택한다.
 *
 * `indexOffset`은 byte offset이 아니라 uint32 index 원소 번호다.
 * 실제 glDrawElements 호출에서는 `indexOffset * sizeof(uint32_t)` byte offset으로 변환된다.
 */
struct MeshFilter
{
    /** @brief AssetManager가 생성/공유하는 GPU Mesh. nullptr이면 그릴 geometry가 없다. */
    std::shared_ptr<Mesh> mesh;

    /** @brief Mesh index buffer에서 시작할 index 원소 offset. 단위: index element 개수. */
    std::size_t indexOffset = 0U;

    /**
     * @brief 그릴 index 원소 개수.
     * @note 현재 렌더 경로에서 0은 전체 Mesh index count를 사용한다는 sentinel 의미로 사용한다.
     */
    std::size_t indexCount = 0U;
};

/**
 * @brief Mesh를 어떤 Shader/Material 상태로 그릴지 지정하는 ECS Component.
 *
 * @details
 * MeshFilter가 "어떤 geometry를" 지정한다면 MeshRenderer는 "어떻게 그릴지"를 지정한다.
 * 실제 OpenGL binding/uniform 설정은 Renderer가 담당한다.
 */
struct MeshRenderer
{
    /** @brief Draw에 사용할 GLSL Program wrapper. nullptr이면 정상 render item으로 사용할 수 없다. */
    std::shared_ptr<Shader> shader;

    /** @brief PBR factor/texture를 보관하는 Material. nullptr이면 호출 경로에 따라 fallback이 필요하다. */
    std::shared_ptr<Material> material;

    /** @brief false이면 RenderSystem이 이 item의 main draw를 건너뛰기 위한 visibility flag. */
    bool visible = true;
};
