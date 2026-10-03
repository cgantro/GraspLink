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
    std::uint32_t m_Framebuffer = 0;
    std::uint32_t m_ColorTexture = 0;
    std::uint32_t m_DepthStencilBuffer = 0;

    int m_Width = 0;
    int m_Height = 0;
    int m_Samples = 4;
};
