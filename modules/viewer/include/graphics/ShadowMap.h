#pragma once

#include <cstdint>

class ShadowMap final
{
public:
    explicit ShadowMap(int size = 2048);
    ~ShadowMap();

    ShadowMap(const ShadowMap&) = delete;
    ShadowMap& operator=(const ShadowMap&) = delete;

    void Begin();
    void End();

    void Bind(std::uint32_t slot) const;

private:
    std::uint32_t m_Framebuffer = 0;
    std::uint32_t m_DepthTexture = 0;

    int m_Size = 2048;

    int m_PreviousFramebuffer = 0;
    int m_PreviousViewport[4]{};
};