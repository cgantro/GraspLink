#pragma once

#include <glm/glm.hpp>

namespace PoseLink
{
struct Transform;
struct Renderable;

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
} // namespace PoseLink
