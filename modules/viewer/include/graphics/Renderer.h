#pragma once

#include <glm/glm.hpp>
#include <memory>

struct MeshFilter;
struct MeshRenderer;
class MultisampleFramebuffer;
class Shader;
class ShadowMap;

/**
 * @brief ECS/Flecs를 알지 않고 OpenGL draw command와 frame pass만 수행하는 렌더링 backend.
 *
 * @details
 * RenderSystem이 "무엇을 그릴지" 선택하면 Renderer는 실제 GPU 상태와 pass 순서를 처리한다.
 * 현재 frame은 다음 순서로 구성된다.
 *
 * `Shadow Pass -> Main Pass(MSAA framebuffer) -> Resolve to default framebuffer`
 *
 * Matrix/좌표 규칙:
 * - model/view/projection/light-space matrix는 GLM column-major 4x4 matrix다.
 * - model matrix의 translation 단위는 Scene/asset world unit이며 현재 HCR scene에서는 meter [m].
 * - cameraPosition/lightDirection은 world-space 기준이다.
 *
 * @todo [FUTURE] light direction, shadow camera 범위, shadow resolution을 설정 객체로 분리한다.
 * @todo [FUTURE] PBR texture 종류가 늘어나면 texture slot 정책을 별도 binding 계층으로 분리한다.
 */
class Renderer
{
public:
    /** @brief 아직 GPU pass resource를 만들지 않은 Renderer 객체를 생성한다. 실제 자원 생성은 Init()에서 수행한다. */
    Renderer();

    /** @brief 소유한 ShadowMap/MSAA framebuffer/shader reference를 정리한다. */
    ~Renderer();

    /**
     * @brief OpenGL global state, MSAA framebuffer, ShadowMap과 shadow shader를 초기화한다.
     * @param framebufferWidth 현재 Window framebuffer 가로 크기 [pixel].
     * @param framebufferHeight 현재 Window framebuffer 세로 크기 [pixel].
     */
    void Init(int framebufferWidth, int framebufferHeight);

    /** @brief Main pass용 MSAA draw target을 바인딩하고 color/depth buffer를 초기화한다. */
    void BeginFrame();

    /** @brief MSAA main-pass color 결과를 default framebuffer로 resolve한다. */
    void EndFrame();

    /**
     * @brief Window framebuffer 크기에 맞춰 내부 render target을 갱신한다.
     * @param width 새 framebuffer width [pixel].
     * @param height 새 framebuffer height [pixel].
     */
    void Resize(int width, int height);

    /**
     * @brief Directional light 관점의 depth-only shadow pass를 시작한다.
     * @note 내부에서 light-space view/projection matrix를 계산하고 ShadowMap framebuffer를 바인딩한다.
     */
    void BeginShadowPass();

    /**
     * @brief 하나의 Mesh 범위를 shadow depth map에 그린다.
     * @param model object local -> world 변환 matrix.
     * @param meshFilter 그릴 GPU Mesh 및 index range.
     */
    void DrawShadow(const glm::mat4& model, const MeshFilter& meshFilter);

    /** @brief shadow framebuffer를 종료하고 이전 framebuffer/viewport 상태를 복원한다. */
    void EndShadowPass();

    /**
     * @brief 하나의 render item을 main pass에 그린다.
     * @param model object local 좌표를 World 좌표로 옮기는 Model Matrix.
     * @param meshFilter GPU Mesh와 index range.
     * @param meshRenderer Shader/Material/visibility 정보.
     * @param view World -> Camera/View 변환 Matrix.
     * @param projection View -> Clip 변환 Perspective Projection Matrix.
     * @param cameraPosition specular 계산 등에 필요한 World camera 위치. 현재 HCR scene에서는 [m].
     */
    void Draw(
        const glm::mat4& model,
        const MeshFilter& meshFilter,
        const MeshRenderer& meshRenderer,
        const glm::mat4& view,
        const glm::mat4& projection,
        const glm::vec3& cameraPosition);

private:
    /** @brief Directional-light depth texture/framebuffer owner. */
    std::unique_ptr<ShadowMap> m_ShadowMap;

    /** @brief Shadow depth pass에 사용하는 shader program. */
    std::shared_ptr<Shader> m_ShadowShader;

    /** @brief Main pass anti-aliasing용 multisample framebuffer owner. */
    std::unique_ptr<MultisampleFramebuffer> m_MSAAFramebuffer;

    /** @brief World-space directional light 방향 기준값. Draw 시 normalize해서 사용할 수 있다. */
    glm::vec3 m_LightDirection{-0.45F, 0.85F, 0.35F};

    /** @brief World position을 directional-light clip space로 변환하는 shadow matrix. */
    glm::mat4 m_LightSpaceMatrix{1.0F};
};
