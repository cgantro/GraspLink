#pragma once

#include <glm/glm.hpp>
#include <memory>

struct MeshFilter;
struct MeshRenderer;
class MultisampleFramebuffer;
class Shader;
class ShadowMap;

/**
 * @brief ECS/Flecs를 알지 않고 OpenGL draw command와 frame pass만 수행한다.
 *
 * @details
 * RenderSystem이 "무엇을 그릴지" 선택하면 Renderer는 실제 GPU 상태를 설정하고 그린다.
 * 현재 frame은 크게 Shadow Pass와 Main Pass로 구성되고, Main Pass는 MSAA framebuffer에
 * 렌더링한 후 default framebuffer로 resolve한다.
 *
 * @todo [FUTURE] light direction, shadow camera 범위, shadow resolution을 Renderer 하드코딩에서 설정 객체로 분리한다.
 * @todo [FUTURE] Material의 PBR texture 종류가 늘어나면 texture slot 정책을 별도 binding 계층으로 분리한다.
 */
class Renderer
{
public:
    Renderer();
    ~Renderer();

    /** @brief OpenGL global state, MSAA framebuffer, ShadowMap과 shadow shader를 초기화한다. */
    void Init(int framebufferWidth, int framebufferHeight);

    /** @brief frame draw target을 바인딩하고 color/depth buffer를 초기화한다. */
    void BeginFrame();

    /** @brief MSAA 결과를 default framebuffer로 resolve한다. */
    void EndFrame();

    /** @brief Window framebuffer 크기에 맞춰 내부 render target을 갱신한다. */
    void Resize(int width, int height);

    /** @brief light 관점 depth-only shadow pass를 시작한다. */
    void BeginShadowPass();

    /** @brief 하나의 Mesh 범위를 shadow depth map에 그린다. */
    void DrawShadow(const glm::mat4& model, const MeshFilter& meshFilter);

    /** @brief shadow pass를 종료하고 이전 framebuffer/viewport를 복원한다. */
    void EndShadowPass();

    /**
     * @brief 하나의 render item을 main pass에 그린다.
     * @param model object local 좌표를 World 좌표로 옮기는 Model Matrix.
     * @param meshFilter GPU Mesh와 index range.
     * @param meshRenderer Shader/Material/visibility 정보.
     * @param view Camera View Matrix.
     * @param projection Camera Projection Matrix.
     * @param cameraPosition specular 계산 등에 필요한 World camera 위치.
     */
    void Draw(
        const glm::mat4& model,
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
