#pragma once

#include <glm/glm.hpp>
#include <memory>

struct MeshFilter;
struct MeshRenderer;
class MultisampleFramebuffer;
class Shader;
class ShadowMap;

/**
 * @brief 선택된 Mesh의 삼각형을 GPU에 그리라고 OpenGL 명령을 보내 그림자와 카메라 화면을 만든다.
 * @details
 * Mesh는 여러 삼각형의 꼭짓점과 꼭짓점 번호를 모은 3D 형상이다. Entity는 장면 안의 한 부품이다. RenderSystem은 장면에서 그릴 Entity와 각 World 위치를 모아 Renderer에 전달한다.
 * Renderer는 Entity의 Mesh, 표면 색과 반사 특성을 담은 Material, 정점 위치와 픽셀 색을 계산하는 Shader를 GPU에 연결해 화면을 만든다.
 * Texture unit은 Shader가 읽을 GPU 이미지를 선택하는 번호 슬롯이다. Renderer는 기본색 Texture를 0번 슬롯에, 그림자 깊이 Texture를 7번 슬롯에 연결한다.
 *
 * 그림자 단계는 광원에서 본 형상을 깊이 이미지에 기록한다. 여기서 깊이는 실제 광원 거리나 카메라 앞뒤값이 아니라 광원 기준 투영 방향에서 표면의 앞뒤 순서를 나타낸다.
 * 화면 단계는 같은 광원 투영으로 표면의 깊이를 계산해 이미지에 저장된 값과 비교한다. 저장된 깊이가 더 앞이면 광선이 가로막힌 것으로 판단해 표면을 어둡게 한다.
 * 카메라 화면에서는 별도로 카메라 앞뒤 깊이를 저장한다. 같은 화면 픽셀에 겹친 표면 중 카메라에 가까운 표면만 표시하는 데 쓰이며 실제 거리와 일정한 비율로 변하지 않는다.
 *
 * MSAA는 픽셀 안 여러 위치에서 삼각형이 덮는지 검사해 가장자리가 차지한 비율을 구하는 방법이다. 이 Renderer는 최대 4곳을 검사하고 EndFrame에서 표본 색을 평균해 창의 한 픽셀로 합친다.
 * Draw Call은 CPU가 GPU에 Mesh의 특정 꼭짓점 번호 구간을 그리라고 보내는 명령이다. 현재 받은 항목을 모두 그리며 화면 밖 물체를 미리 제외하거나 순서를 정렬하지 않는다.
 * 삼각형의 앞면만 남기는 기능은 꺼져 있어 앞면과 뒷면을 모두 그린다. 소유한 화면 저장소·Texture·Shader는 객체 파괴 때 GPU에서 해제하며, 이때 OpenGL 명령 실행 환경(context)이 현재 스레드에서 활성화되어야 한다.
 * @todo [FUTURE] 하드코딩된 광원 방향·그림자 카메라 범위·해상도를 설정 객체로 분리한다.
 * @todo [FUTURE] 물리 재질 계산에 쓰는 이미지 종류가 늘어나면 이미지 슬롯 배정 규칙을 별도 관리로 옮긴다.
 */
class Renderer
{
public:
    Renderer();
    ~Renderer();

    /**
     * @brief 한 프레임의 색·깊이 저장소와 그림자 계산에 쓸 광원 변환을 준비한다.
     * @param framebufferWidth 화면에 실제 표시할 픽셀 너비 [pixel].
     * @param framebufferHeight 화면에 실제 표시할 픽셀 높이 [pixel].
     * @details Framebuffer는 창 자체가 아니라 GPU가 색이나 깊이를 기록하는 출력 저장소다. 화면용 저장소는 픽셀마다 최대 4개 색 표본과 카메라 깊이를 담는다.
     * 그림자 이미지는 광원 기준 투영 깊이를 담는 2048×2048 픽셀 이미지다. 광원 방향은 길이 1인 방향 벡터로 맞추고 작업 공간 중심을 향한다.
     * 행렬은 점의 좌표를 다른 기준으로 바꾸는 숫자 표다. 장면(World) 좌표에서 광원 시점의 Clip 좌표로 바꾸는 행렬을 깊이 기록과 화면의 비교에서 함께 쓴다.
     */
    void Init(int framebufferWidth, int framebufferHeight);

    /** @brief 화면용 다중 표본 버퍼를 그리기 대상으로 선택하고 이전 프레임 색과 깊이를 비운다. */
    void BeginFrame();

    /** @brief 여러 표본으로 얻은 색을 Window가 표시할 픽셀 색 하나로 합쳐 프레임을 마친다. */
    void EndFrame();

    /**
     * @brief 창 픽셀 크기에 맞춰 가장자리를 부드럽게 그릴 화면 버퍼의 크기를 바꾼다.
     * @param width 새 화면 너비 [pixel].
     * @param height 새 화면 높이 [pixel].
     */
    void Resize(int width, int height);

    /** @brief 화면에서 장면을 그릴 픽셀 사각형을 지정한다. */
    void SetSceneViewport(int x, int y, int width, int height);

    /** @brief 광원 기준으로 표면 깊이만 그림자 이미지에 기록하기 시작한다. */
    void BeginShadowPass();

    /**
     * @brief 지정한 Mesh 구간을 광원 기준 투영 깊이 이미지에 기록한다.
     * @param model Mesh 기준 좌표를 장면 전체 좌표로 바꾸는 행렬.
     * @param meshFilter GPU에 올린 Mesh와 그릴 꼭짓점 연결 범위를 담은 Entity 자료.
     */
    void DrawShadow(const glm::mat4& model, const MeshFilter& meshFilter);

    /** @brief 그림자 그리기를 끝내고 그림자 그리기 전 화면 버퍼와 그릴 영역을 복구한다. */
    void EndShadowPass();

    /**
     * @brief 재질의 색과 조명으로 Mesh 표면 색을 계산해 카메라 화면에 그린다.
     * @param model Mesh 기준 좌표를 장면 전체 좌표로 바꾸는 행렬.
     * @param meshFilter GPU에 올린 Mesh와 그릴 꼭짓점 연결 범위를 담은 Entity 자료.
     * @param meshRenderer 표시 여부와 표면 재질, 그리기 프로그램 참조를 담은 Entity 자료.
     * @param view World 좌표를 카메라 위치와 방향 기준 좌표로 바꾸는 행렬.
     * @param projection 가까운 물체를 크게, 먼 물체를 작게 보이게 해 Clip 좌표와 깊이를 만드는 행렬.
     * @param cameraPosition 장면 좌표에서 본 카메라 위치 [m].
     * @details Model 행렬은 Mesh 자체 기준(Local) 좌표를 부모와 장면 변환이 적용된 World 좌표로 바꾼다. View는 World 좌표를 카메라 기준으로 바꾼다.
     * Projection은 원근을 적용해 Clip 좌표를 만들고, GPU는 이를 화면 위치와 카메라 깊이로 나눈다. 이 깊이는 실제 거리와 같은 간격으로 변하지 않는다.
     * 재질 색·금속성·거칠기 계수·기본색 이미지·카메라와 광원 위치·그림자 깊이 이미지를 Shader 입력에 전달한다.
     * Robot.glsl은 ACES 곡선으로 여러 빛을 더한 RGB를 화면 범위에 압축하고 1/2.2 거듭제곱으로 표시용 색으로 바꾼다. 조명식은 금속성과 거칠기를 반영한 간단한 근사다.
     * indexOffset은 꼭짓점 번호 개수 단위다. GPU 명령에 넘길 때는 번호 하나의 크기인 4 byte를 곱한다. 필요한 Mesh, Shader 또는 재질 참조가 없으면 그리지 않는다.
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

    // Shader는 GPU 프로그램이다. 이 Shader는 광원 기준 투영 깊이만 그림자 이미지에 기록한다.
    std::shared_ptr<Shader> m_ShadowShader;

    std::unique_ptr<MultisampleFramebuffer> m_MSAAFramebuffer;

    // World 좌표에서 표면으로부터 광원을 향하는 방향 벡터. Init에서 길이를 1로 맞춘다.
    glm::vec3 m_LightDirection{-0.45F, 0.85F, 0.35F};

    // World 좌표를 광원 기준 Clip 좌표로 바꾸는 행렬. 깊이 기록과 화면의 가림 판정이 같은 변환을 쓴다.
    glm::mat4 m_LightSpaceMatrix{1.0F};

    // 메인 카메라 장면을 표시할 영역이다. 창 전체에서 UI 영역을 제외한 픽셀 범위를 사용한다.
    glm::ivec4 m_SceneViewport{0, 0, 0, 0};
};
