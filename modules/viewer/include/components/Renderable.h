#pragma once

#include <cstddef>
#include <memory>

namespace PoseLink
{
class Mesh;
class Shader;
class Material;

/**
 * 한 entity를 그리는 데 필요한 GPU resource의 참조다.
 * 여러 node가 같은 mesh, shader, material을 사용할 수 있으므로 resource는
 * shared_ptr로 공유하고 entity는 소유권을 직접 관리하지 않는다.
 */
struct Renderable
{
    std::shared_ptr<Mesh> mesh;
    std::shared_ptr<Shader> shader;
    std::shared_ptr<Material> material;
    std::size_t indexOffset = 0U;
    std::size_t indexCount = 0U;
};
} // namespace PoseLink
