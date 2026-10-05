#pragma once

#include <cstdint>

// 역할: 광원에서 본 가장 가까운 표면의 깊이를 저장해 그림자 판정에 사용.
// 소유: 깊이 texture와 framebuffer를 생성·해제. 소멸까지 GL context가 살아 있어야 함.
class ShadowMap final
{
public:
    // 입력: 정사각형 한 변의 크기 [pixel], 양수만 허용.
    explicit ShadowMap(int size = 2048);

    ~ShadowMap();

    ShadowMap(const ShadowMap&) = delete;
    ShadowMap& operator=(const ShadowMap&) = delete;

    // 반영: 그림자용 framebuffer와 viewport를 설정하고 깊이를 초기화.
    void Begin();

    // 복원: Begin 전 framebuffer와 viewport. 표면 깊이 보정은 끔.
    void End();

    // 입력: Shader가 깊이 texture를 읽을 texture unit 번호.
    void Bind(std::uint32_t slot) const;

private:
    std::uint32_t m_Framebuffer = 0;

    std::uint32_t m_DepthTexture = 0;

    // 단위: 정사각형 한 변의 크기 [pixel].
    int m_Size = 2048;

    // 복원: Begin 전 렌더 대상과 {x, y, width, height} [pixel].
    int m_PreviousFramebuffer = 0;

    int m_PreviousViewport[4]{};
};
