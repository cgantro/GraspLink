#pragma once

#include <cstdint>

/**
 * @brief MSAA 렌더 타깃과 resolve 과정을 관리하는 Framebuffer wrapper.
 *
 * @details
 * MSAA는 하나의 화면 pixel 내부를 여러 sample로 평가해 polygon edge의 계단 현상을 줄인다.
 * multisample framebuffer에 직접 그린 결과는 화면에 바로 표시할 수 없으므로 프레임 끝에서
 * ResolveToDefault()가 glBlitFramebuffer를 사용해 default framebuffer로 resolve한다.
 *
 * @todo [FUTURE] sample count를 Renderer 하드코딩 값이 아니라 graphics settings로 이동한다.
 */
/*
 * [추가 그래픽스 용어 설명]
 * - Framebuffer: "이번 draw 결과를 어디에 저장할지"를 정하는 렌더링 대상.
 * - Default framebuffer: Window가 화면에 표시하는 기본 렌더링 대상.
 * - Off-screen framebuffer: 화면에 바로 보여주지 않고 중간 결과를 저장하는 별도 렌더링 대상.
 * - MSAA(Multisample Anti-Aliasing): 한 pixel을 여러 sample로 평가해 경계의 계단 현상을 줄이는 방식.
 * - Sample: 한 pixel 안에서 색/깊이를 평가하는 하위 지점.
 * - Resolve: 여러 sample 결과를 최종 한 pixel 색으로 합치는 과정.
 * - Color Texture: 렌더링된 색을 저장하는 GPU texture.
 * - Depth/Stencil Buffer: 깊이 테스트와 stencil 테스트에 쓰는 보조 GPU buffer.
 *
 * width/height는 framebuffer pixel 크기이고 samples는 pixel당 sample 개수다.
 */
class MultisampleFramebuffer final
{
public:
    /** @brief 지정 해상도와 sample count로 multisample framebuffer를 생성한다. */
    MultisampleFramebuffer(int width, int height, int samples = 4);

    /** @brief OpenGL framebuffer/texture/renderbuffer를 해제한다. */
    ~MultisampleFramebuffer();

    MultisampleFramebuffer(const MultisampleFramebuffer&) = delete;
    MultisampleFramebuffer& operator=(const MultisampleFramebuffer&) = delete;

    /** @brief 이 framebuffer를 draw target으로 바인딩하고 viewport를 맞춘다. */
    void Bind() const;

    /** @brief Window 크기 변경 시 multisample storage를 재생성한다. */
    void Resize(int width, int height);

    /** @brief multisample color 결과를 default framebuffer로 resolve한다. */
    void ResolveToDefault() const;

    int GetWidth() const { return m_Width; }
    int GetHeight() const { return m_Height; }
    int GetSamples() const { return m_Samples; }

private:
    /** @brief 현재 width/height/sample 설정으로 OpenGL 자원을 생성한다. */
    void Create();

    /** @brief 생성된 OpenGL 자원을 삭제하고 ID를 0으로 초기화한다. */
    void Destroy();

private:
    // OpenGL framebuffer object ID.
    std::uint32_t m_Framebuffer = 0;

    // MSAA color 결과를 저장하는 multisample texture ID.
    std::uint32_t m_ColorTexture = 0;

    // depth/stencil 값을 저장하는 renderbuffer ID.
    std::uint32_t m_DepthStencilBuffer = 0;

    // framebuffer 크기 [pixel].
    int m_Width = 0;
    int m_Height = 0;

    // pixel당 sample 개수. 기본값 4는 4x MSAA를 의미한다.
    int m_Samples = 4;
};
