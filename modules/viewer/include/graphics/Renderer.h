#pragma once

#include <glm/glm.hpp>
#include <memory>

struct MeshFilter;
struct MeshRenderer;
class MultisampleFramebuffer;
class Shader;
class ShadowMap;

/**
 * @brief 선택된 Mesh를 그림자와 최종 화면에 그리는 OpenGL 렌더러.
 * @details
 * RenderSystem은 Flecs에서 보이는 Entity와 World 변환을 모아 전달하고, Renderer는 전달받은 값으로 OpenGL 상태와 draw 명령을 처리한다.
 * Shadow Pass는 광원에서 본 깊이를 depth texture에 기록해 Main Pass의 가림 판정에 쓴다. Main Pass는 카메라가 보는 색과 깊이를 화면용 framebuffer에 그린다.
 * 이 renderer의 MSAA target은 픽셀마다 최대 4개 sample을 저장해 삼각형 경계 coverage를 부드럽게 표현한다. EndFrame의 resolve는 각 픽셀 sample들을 Window 기본 framebuffer의 색으로 합친다.
 * CPU가 한 Mesh index 구간을 그리도록 GPU에 보내는 명령이 Draw Call이다. 현재 구현은 전달된 항목을 그리며 시야 밖 물체를 거르는 frustum culling이나 draw 정렬을 하지 않는다.
 * Depth Test는 같은 픽셀에서 카메라에 가까운 fragment를 남기는 깊이 비교다. 이 코드에서는 활성화하지만 face culling은 활성화하지 않아 뒷면 삼각형도 rasterize한다.
 * Main Pass의 조명 shader는 앞서 만든 ShadowMap을 샘플링할 수 있다.
 * 이 객체가 소유한 framebuffer, texture, shader의 GL 핸들은 소멸할 때 해제되므로 GL context가 살아 있는 동안 만들어지고 파괴되어야 한다.
 * @todo [FUTURE] 하드코딩된 광원 방향·그림자 카메라 범위·해상도를 설정 객체로 분리한다.
 * @todo [FUTURE] PBR 텍스처 종류가 늘어나면 텍스처 슬롯 배정 규칙을 별도 바인딩 계층으로 분리한다.
 */
class Renderer
{
public:
    Renderer();
    ~Renderer();

    /**
     * @brief 프레임 렌더링에 필요한 GL 상태와 GPU 자원을 준비한다.
     * @param framebufferWidth Window framebuffer 너비 [pixel].
     * @param framebufferHeight Window framebuffer 높이 [pixel].
     * @details 화면용 target은 4 samples, 그림자 depth texture는 2048×2048로 만든다. 고정 조명 방향을 정규화하고 작업 공간을 향하는 light-space 변환을 저장한다.
     */
    void Init(int framebufferWidth, int framebufferHeight);

    /** @brief MSAA framebuffer를 대상으로 바인딩하고 색 및 깊이 버퍼를 지운다. */
    void BeginFrame();

    /** @brief MSAA 색 결과를 Window 기본 framebuffer로 resolve해 프레임을 마친다. */
    void EndFrame();

    /**
     * @brief Window 크기 변경에 맞춰 화면용 MSAA framebuffer 크기를 갱신한다.
     * @param width 새 framebuffer 너비 [pixel].
     * @param height 새 framebuffer 높이 [pixel].
     */
    void Resize(int width, int height);

    /** @brief 조명 시점의 depth-only 그림자 패스를 시작하고 shader에 light-space 행렬을 전달한다. */
    void BeginShadowPass();

    /**
     * @brief Mesh의 지정된 index 구간을 그림자 깊이 맵에 기록한다.
     * @param model Mesh의 Local 좌표를 World 좌표로 옮기는 행렬.
     * @param meshFilter GPU Mesh와 그릴 index 구간을 가리키는 ECS component.
     */
    void DrawShadow(const glm::mat4& model, const MeshFilter& meshFilter);

    /** @brief 그림자 shader를 해제하고 ShadowMap 이전의 framebuffer와 viewport를 복원한다. */
    void EndShadowPass();

    /**
     * @brief Mesh를 카메라 화면에 재질과 조명을 적용해 그린다.
     * @param model Mesh의 Local 좌표를 World 좌표로 옮기는 행렬.
     * @param meshFilter GPU Mesh와 그릴 index 구간을 가리키는 ECS component.
     * @param meshRenderer 표시 여부와 shader 및 material 참조를 담은 ECS component.
     * @param view World 좌표를 카메라 View 좌표로 옮기는 행렬.
     * @param projection View 좌표를 Clip 좌표로 옮기는 투영 행렬.
     * @param cameraPosition World 좌표계의 카메라 위치 [m].
     * @details Model, View, Projection 행렬은 정점의 Local 위치를 Clip 공간으로 옮긴다. Material의 base color, metallic, roughness 계수와 선택적 texture, 카메라/조명 값, 그림자 깊이 texture를 shader에 설정한다. Robot.glsl은 이를 단순화한 metallic/roughness 조명에 사용하고 ACES tone mapping 뒤 2.2 감마 보정을 적용한다. indexOffset은 시작 index 번호이므로 GL draw 전에 uint32 index의 byte offset으로 바꾼다. Mesh, shader 또는 material 참조가 비어 있으면 draw를 생략한다.
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

    // Shadow Pass에서 위치와 light-space 깊이만 기록하는 shader.
    std::shared_ptr<Shader> m_ShadowShader;

    std::unique_ptr<MultisampleFramebuffer> m_MSAAFramebuffer;

    // 표면에서 광원 쪽을 향하는 World 방향 벡터. Init에서 정규화한다.
    glm::vec3 m_LightDirection{-0.45F, 0.85F, 0.35F};

    // World 좌표를 조명 기준 Clip 좌표로 바꾸며 Shadow Pass와 Main Pass가 함께 사용한다.
    glm::mat4 m_LightSpaceMatrix{1.0F};
};
