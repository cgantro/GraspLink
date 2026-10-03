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
struct MeshFilter
{
    std::shared_ptr<Mesh> mesh;
    std::size_t indexOffset = 0U;
    std::size_t indexCount = 0U;
};

/**
 * @brief Mesh를 어떤 Shader/Material로 그릴지 지정하는 ECS Component.
 * @todo [FUTURE] render layer, cast/receive shadow, culling flag가 필요해지면 이 Component를 확장한다.
 */
struct MeshRenderer
{
    std::shared_ptr<Shader> shader;
    std::shared_ptr<Material> material;
    bool visible = true;
};
