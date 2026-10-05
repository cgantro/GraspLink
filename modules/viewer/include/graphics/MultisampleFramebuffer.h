#pragma once

#include <cstdint>

// 역할: pixel 안의 여러 지점에 그려 경계의 계단 현상을 줄인 뒤 Window로 합쳐 복사.
// 소유: framebuffer, 색 texture, 깊이/stencil buffer. 소멸까지 GL context가 살아 있어야 함.
class MultisampleFramebuffer final
{
public:
    // 입력: 양수 크기 [pixel]와 pixel당 sample 수. sample 수는 GPU 최대값으로 제한.
    MultisampleFramebuffer(int width, int height, int samples = 4);

    ~MultisampleFramebuffer();

    MultisampleFramebuffer(const MultisampleFramebuffer&) = delete;
    MultisampleFramebuffer& operator=(const MultisampleFramebuffer&) = delete;

    // 반영: 렌더 대상과 viewport를 이 저장소 크기로 설정.
    void Bind() const;

    // 입력: 새 크기 [pixel]. 0 이하와 저장소가 있는 같은 크기는 무시.
    // 실패: 기존·새 GPU 저장소 모두 해제. 같은 양수 크기로도 다시 시도 가능.
    void Resize(int width, int height);

    // 출력: 여러 sample의 색을 Window의 기본 framebuffer로 합쳐 복사.
    void ResolveToDefault() const;

    int GetWidth() const { return m_Width; }
    int GetHeight() const { return m_Height; }
    int GetSamples() const { return m_Samples; }

private:
    void Create();

    void Destroy();

private:
    std::uint32_t m_Framebuffer = 0;

    std::uint32_t m_ColorTexture = 0;

    std::uint32_t m_DepthStencilBuffer = 0;

    // 단위: 저장소 크기 [pixel].
    int m_Width = 0;
    int m_Height = 0;

    // 단위: pixel당 sample 수.
    int m_Samples = 4;
};
