#pragma once

#include <cstdint>

/**
 * @brief 다중 sample 렌더링 결과를 기본 framebuffer로 resolve하는 MSAA 대상.
 * @details 색상 attachment는 multisample RGBA8 texture이고 깊이와 stencil attachment는 multisample DEPTH24_STENCIL8 renderbuffer다.
 * 픽셀마다 여러 위치를 sample하면 도형 경계에서 픽셀의 부분적인 덮임을 표현해 계단 현상을 줄일 수 있다.
 * ResolveToDefault()는 각 픽셀의 sample 색상을 합성해 기본 framebuffer로 복사한다. 기본 framebuffer는 이 대상과
 * 같은 크기여야 한다. 모든 메서드와 소멸자는 현재 스레드에 유효한 OpenGL context가 있을 때 호출해야 한다.
 */
class MultisampleFramebuffer final
{
public:
    /**
     * @brief 지정한 픽셀 크기와 sample 수로 multisample framebuffer를 만든다.
     * @param width 너비(pixel). 양수여야 한다.
     * @param height 높이(pixel). 양수여야 한다.
     * @param samples 픽셀당 요청 sample 수. 기본값은 4이며 GL_MAX_SAMPLES보다 크면 지원 상한으로 낮춘다.
     * @throws std::invalid_argument 너비나 높이가 양수가 아닐 때.
     * @throws std::runtime_error 제한 적용 후 sample 수가 2보다 작거나 framebuffer가 불완전할 때.
     */
    MultisampleFramebuffer(int width, int height, int samples = 4);

    /** @brief 소유한 GPU 객체를 해제한다. 현재 유효한 OpenGL context가 필요하다. */
    ~MultisampleFramebuffer();

    MultisampleFramebuffer(const MultisampleFramebuffer&) = delete;
    MultisampleFramebuffer& operator=(const MultisampleFramebuffer&) = delete;

    /** @brief 이 framebuffer를 렌더 대상으로 바인딩하고 전체 크기로 viewport를 설정한다.
     * @details GL_FRAMEBUFFER 바인딩과 viewport를 변경하며, 이전 viewport를 복원하지 않는다.
     */
    void Bind() const;

    /**
     * @brief 색상 texture와 depth/stencil renderbuffer를 새 픽셀 크기로 재생성한다.
     * @param width 새 너비(pixel).
     * @param height 새 높이(pixel).
     * @details 너비나 높이가 0 이하이면 아무 작업도 하지 않는다. 크기가 같고 framebuffer가 존재하면 재생성하지 않는다.
     * 재생성할 때 GPU 객체를 해제하고 새 attachment를 만든다. 새 framebuffer가 불완전하면 객체를 정리하고 예외를 던진다.
     * 이때 새 크기는 저장되어 있고 framebuffer는 없으므로 유효한 크기로 다시 호출해 복구할 수 있다.
     * @throws std::runtime_error 재생성한 framebuffer가 불완전할 때.
     */
    void Resize(int width, int height);

    /**
     * @brief multisample 색상을 기본 framebuffer(0)에 resolve해 같은 픽셀 크기로 복사한다.
     * @details 깊이와 stencil은 복사하지 않는다. 읽기 framebuffer는 이 객체, 쓰기 framebuffer는 0으로 설정하고
     * 작업 후 GL_FRAMEBUFFER를 0으로 바인딩한다. viewport는 변경하지 않는다. 기본 framebuffer와 크기를 맞춰야 하며
     * 현재 유효한 OpenGL context가 필요하다.
     */
    void ResolveToDefault() const;

    /** @brief 생성 또는 마지막 Resize에 지정된 너비(pixel)를 반환한다. */
    int GetWidth() const { return m_Width; }
    /** @brief 생성 또는 마지막 Resize에 지정된 높이(pixel)를 반환한다. */
    int GetHeight() const { return m_Height; }
    /** @brief GPU 상한 적용 후 사용되는 픽셀당 sample 수를 반환한다. */
    int GetSamples() const { return m_Samples; }

private:
    // 동일한 크기와 sample 수를 쓰는 색상 및 깊이/stencil attachment를 만들고 완전성을 확인한다.
    void Create();

    // 소유한 GL 객체를 해제하고 핸들을 0으로 초기화한다.
    void Destroy();

private:
    std::uint32_t m_Framebuffer = 0;

    std::uint32_t m_ColorTexture = 0;

    std::uint32_t m_DepthStencilBuffer = 0;

    // 현재 attachment의 크기(pixel).
    int m_Width = 0;
    int m_Height = 0;

    // 색상과 깊이 attachment에 공통으로 적용하는 픽셀당 sample 수.
    int m_Samples = 4;
};
