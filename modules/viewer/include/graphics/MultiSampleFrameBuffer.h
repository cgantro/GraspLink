#pragma once

#include <cstdint>

/**
 * @brief MSAA 렌더 타깃과 default framebuffer resolve 과정을 관리하는 OpenGL Framebuffer wrapper.
 *
 * @details
 * MSAA는 하나의 화면 pixel 내부를 여러 sample로 평가해 polygon edge aliasing을 줄인다.
 * multisample framebuffer에 직접 렌더한 color buffer는 화면에 바로 표시하지 않고,
 * frame 끝에서 ResolveToDefault()가 glBlitFramebuffer로 default framebuffer에 resolve한다.
 *
 * width/height 단위는 framebuffer pixel이고 samples는 pixel당 sample 개수다.
 *
 * @todo [FUTURE] sample count를 Renderer 하드코딩 값이 아니라 graphics settings로 이동한다.
 */
class MultisampleFramebuffer final
{
public:
    /**
     * @brief 지정 해상도와 sample count로 multisample framebuffer를 생성한다.
     * @param width framebuffer 가로 크기 [pixel].
     * @param height framebuffer 세로 크기 [pixel].
     * @param samples pixel당 multisample 개수. 기본 4x MSAA.
     */
    MultisampleFramebuffer(int width, int height, int samples = 4);

    /** @brief 소유한 OpenGL framebuffer/texture/renderbuffer를 해제한다. */
    ~MultisampleFramebuffer();

    /** @brief OpenGL object ownership 중복을 막기 위해 copy construction을 금지한다. */
    MultisampleFramebuffer(const MultisampleFramebuffer&) = delete;

    /** @brief OpenGL object ownership 중복을 막기 위해 copy assignment를 금지한다. */
    MultisampleFramebuffer& operator=(const MultisampleFramebuffer&) = delete;

    /** @brief 이 framebuffer를 draw target으로 바인딩하고 viewport를 저장된 width/height[pixel]로 맞춘다. */
    void Bind() const;

    /**
     * @brief Window framebuffer 크기 변경 시 multisample storage를 재생성한다.
     * @param width 새 가로 크기 [pixel].
     * @param height 새 세로 크기 [pixel].
     */
    void Resize(int width, int height);

    /** @brief multisample color 결과를 현재 default framebuffer로 resolve한다. */
    void ResolveToDefault() const;

    /** @return 현재 framebuffer 가로 크기 [pixel]. */
    int GetWidth() const { return m_Width; }

    /** @return 현재 framebuffer 세로 크기 [pixel]. */
    int GetHeight() const { return m_Height; }

    /** @return pixel당 MSAA sample 개수. */
    int GetSamples() const { return m_Samples; }

private:
    /** @brief 현재 width/height/sample 설정으로 OpenGL 자원을 생성한다. */
    void Create();

    /** @brief 생성된 OpenGL 자원을 삭제하고 object ID를 0으로 초기화한다. */
    void Destroy();

private:
    /** @brief MSAA draw target OpenGL framebuffer object ID. */
    std::uint32_t m_Framebuffer = 0;

    /** @brief multisample color attachment texture object ID. */
    std::uint32_t m_ColorTexture = 0;

    /** @brief depth/stencil multisample renderbuffer object ID. */
    std::uint32_t m_DepthStencilBuffer = 0;

    /** @brief current framebuffer width [pixel]. */
    int m_Width = 0;

    /** @brief current framebuffer height [pixel]. */
    int m_Height = 0;

    /** @brief pixel당 sample 개수. */
    int m_Samples = 4;
};
