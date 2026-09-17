#pragma once

#include <cstddef>
#include <memory>

class Mesh;
class Shader;
class Material;

// 렌더 시스템이 사용할 메시와 primitive 범위를 보관한다.
struct MeshFilter
{
    std::shared_ptr<Mesh> mesh;
    std::size_t indexOffset = 0U;
    std::size_t indexCount = 0U;
};

// 렌더 시스템이 사용할 셰이더, 머티리얼, 표시 여부를 보관한다.
struct MeshRenderer
{
    std::shared_ptr<Shader> shader;
    std::shared_ptr<Material> material;
    bool visible = true;
};
