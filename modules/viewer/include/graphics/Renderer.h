#pragma once

#include <glm/glm.hpp>

struct MeshFilter;
struct MeshRenderer;

// Flecs나 계층 구조를 알지 않고 GPU draw command만 실행한다.
class Renderer
{
public:
    Renderer();
    ~Renderer();

    void Init();
    void BeginFrame();
    void Draw(const glm::mat4& model,
              const MeshFilter& meshFilter,
              const MeshRenderer& meshRenderer,
              const glm::mat4& view,
              const glm::mat4& projection);
};
