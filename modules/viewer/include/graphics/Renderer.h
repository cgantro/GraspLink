#pragma once

#include <glm/glm.hpp>
#include <memory>

namespace grasplink::rendering
{
struct MeshFilter;
struct MeshRenderer;
}

namespace grasplink::graphics
{

class MultisampleFramebuffer;

class Renderer
{
public:
    Renderer();
    ~Renderer();

    void Init(int framebufferWidth, int framebufferHeight);
    void BeginFrame();
    void EndFrame();
    void Resize(int width, int height);
    void SetSceneViewport(int x, int y, int width, int height);
    void Draw(
        const glm::mat4& model,
        const ::grasplink::rendering::MeshFilter& meshFilter,
        const ::grasplink::rendering::MeshRenderer& meshRenderer,
        const glm::mat4& view,
        const glm::mat4& projection,
        const glm::vec3& cameraPosition);

private:
#ifndef __EMSCRIPTEN__
    std::unique_ptr<MultisampleFramebuffer> m_MSAAFramebuffer;
#else
    int m_FramebufferWidth = 0;
    int m_FramebufferHeight = 0;
#endif
    glm::vec3 m_LightDirection{-0.45F, 0.85F, 0.35F};
    glm::ivec4 m_SceneViewport{0, 0, 0, 0};
};

} // namespace grasplink::graphics
