#pragma once

#include <cstddef>
#include <memory>

#include <glm/glm.hpp>

struct Transform;
class Mesh;
class Shader;
class Material;

// 렌더러가 엔티티에서 읽는 GPU 리소스 참조다.
struct Renderable
{
    std::shared_ptr<Mesh> mesh;
    std::shared_ptr<Shader> shader;
    std::shared_ptr<Material> material;
    std::size_t indexOffset = 0U;
    std::size_t indexCount = 0U;
};

/** OpenGL context에 의존하는 실제 draw call만 담당한다. */
class Renderer
{
public:
    Renderer();
    ~Renderer();

    void Init();
    void BeginFrame();
    void Draw(const Transform& transform, const Renderable& renderable,
              const glm::mat4& view, const glm::mat4& projection);
};
