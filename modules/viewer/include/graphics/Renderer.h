#pragma once

#include <glm/glm.hpp>
#include <memory>

struct MeshFilter;
struct MeshRenderer;
class MultisampleFramebuffer;
class Shader;
class ShadowMap;

// 역할: RenderSystem이 고른 Mesh 범위를 GPU에 그림.
// 흐름: 광원 기준 깊이 → 화면 색 → 여러 sample을 합쳐 Window에 복사.
// 수명: 소유한 GPU 객체가 해제될 때까지 GL context가 살아 있어야 함.
class Renderer
{
public:
    Renderer();
    ~Renderer();

    // 입력: Window framebuffer 크기 [pixel]. 렌더 대상과 그림자 저장소 생성.
    void Init(int framebufferWidth, int framebufferHeight);

    // 반영: 화면용 렌더 대상을 설정하고 색·깊이를 초기화.
    void BeginFrame();

    // 출력: 화면용 렌더 결과를 Window 기본 framebuffer에 복사.
    void EndFrame();

    // 입력: 새 framebuffer 크기 [pixel].
    void Resize(int width, int height);

    // 반영: 광원 기준 깊이를 그릴 저장소와 Shader를 설정.
    void BeginShadowPass();

    // 입력: Local → World 행렬과 Mesh의 index 범위.
    void DrawShadow(const glm::mat4& model, const MeshFilter& meshFilter);

    // 복원: 그림자를 그리기 전 framebuffer와 viewport.
    void EndShadowPass();

    // 좌표: model은 Local → World, view는 World → View, projection은 View → Clip.
    // 입력: cameraPosition은 World 위치 [m], Mesh 범위는 index 개수·시작 번호.
    // 참조: Mesh·Shader·Material은 호출자가 소유하며 이 함수에서는 보관하지 않음.
    void Draw(
        const glm::mat4& model,
        const MeshFilter& meshFilter,
        const MeshRenderer& meshRenderer,
        const glm::mat4& view,
        const glm::mat4& projection,
        const glm::vec3& cameraPosition);

private:
    std::unique_ptr<ShadowMap> m_ShadowMap;

    // 공유: 그림자 draw가 사용하는 깊이 전용 Shader.
    std::shared_ptr<Shader> m_ShadowShader;

    std::unique_ptr<MultisampleFramebuffer> m_MSAAFramebuffer;

    // 좌표: 표면에서 광원 쪽을 향하는 World 단위벡터. 빛이 진행하는 방향의 반대.
    glm::vec3 m_LightDirection{-0.45F, 0.85F, 0.35F};

    // 좌표: World → 광원 기준 Clip 변환.
    glm::mat4 m_LightSpaceMatrix{1.0F};
};
