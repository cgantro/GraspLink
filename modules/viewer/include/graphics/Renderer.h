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
/*
 * [추가 그래픽스 용어 설명]
 * - Render Pass: 한 frame을 만들기 위해 목적별로 나눈 렌더링 단계.
 * - Shadow Pass: light 관점에서 depth만 그려 그림자 판정용 texture를 만드는 단계.
 * - Main Pass: 실제 화면에 보일 색/조명 결과를 그리는 단계.
 * - Draw Call: CPU가 GPU에 "이 Mesh 범위를 이 상태로 그려라"라고 내리는 한 번의 draw 명령.
 * - Model Matrix: object local 좌표를 World 좌표로 바꾸는 행렬.
 * - View Matrix: World 좌표를 Camera 좌표로 바꾸는 행렬.
 * - Projection Matrix: Camera 좌표를 화면 투영 좌표로 바꾸는 행렬.
 * - Light-space Matrix: World 좌표를 light가 보는 좌표/투영으로 변환하는 행렬.
 * - Depth Test: 카메라에 더 가까운 fragment만 화면에 남기기 위한 깊이 비교.
 * - Culling: 보이지 않는 면/물체를 그리지 않아 비용을 줄이는 처리.
 *
 * Renderer는 ECS에서 Entity를 찾지 않는다. RenderSystem이 선택한 데이터를 받아 GPU 명령만 수행한다.
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
    // Shadow depth texture/framebuffer의 lifetime owner.
    std::unique_ptr<ShadowMap> m_ShadowMap;

    // Shadow pass에서 여러 draw가 공유하는 depth-only Shader.
    std::shared_ptr<Shader> m_ShadowShader;

    // Main pass의 multisample color/depth target owner.
    std::unique_ptr<MultisampleFramebuffer> m_MSAAFramebuffer;

    // Directional light가 비추는 방향을 나타내는 World-space 방향벡터. 물리 단위는 없다.
    glm::vec3 m_LightDirection{-0.45F, 0.85F, 0.35F};

    // World position을 light의 clip space로 변환하는 4x4 행렬.
    glm::mat4 m_LightSpaceMatrix{1.0F};
};
