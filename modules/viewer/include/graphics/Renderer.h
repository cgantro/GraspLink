#pragma once

#include <glm/glm.hpp>
#include <memory>
struct MeshFilter;
struct MeshRenderer;
class MultisampleFramebuffer;
class Shader;
class ShadowMap;
// Flecs나 계층 구조를 알지 않고 GPU draw command만 실행한다.
class Renderer
{
public:
    Renderer();
    ~Renderer();

    void Init(int framebufferWidth, int framebufferHeight);
    void BeginFrame();
    void EndFrame();
    void Resize(int width, int height);

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
    std::unique_ptr<MultisampleFramebuffer> m_MSAAFramebuffer;
    
    glm::vec3 m_LightDirection{-0.45F, 0.85F, 0.35F};
    glm::mat4 m_LightSpaceMatrix{1.0F};
};
