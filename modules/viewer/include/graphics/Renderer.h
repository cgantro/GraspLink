#pragma once

#include <glm/glm.hpp>
#include <memory>
struct MeshFilter;
struct MeshRenderer;
class Shader;
class ShadowMap;
// Flecs나 계층 구조를 알지 않고 GPU draw command만 실행한다.
class Renderer
{
public:
    Renderer();
    ~Renderer();

    void Init();
    void BeginFrame();

    void BeginShadowPass();
    void DrawShadow(
        const glm::mat4& model,
        const MeshFilter& meshFilter);
    void EndShadowPass();

    void Draw(const glm::mat4& model,
              const MeshFilter& meshFilter,
              const MeshRenderer& meshRenderer,
              const glm::mat4& view,
              const glm::mat4& projection,
              const glm::vec3& cameraPosition);
private:
    std::unique_ptr<ShadowMap> m_ShadowMap;
    std::shared_ptr<Shader> m_ShadowShader;

    glm::vec3 m_LightDirection{-0.45F, 0.85F, 0.35F};
    glm::mat4 m_LightSpaceMatrix{1.0F};
};
