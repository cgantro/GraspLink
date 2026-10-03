#pragma once

#include <cstdint>

class MultisampleFramebuffer final
{
public:
    MultisampleFramebuffer(int width, int height, int samples = 4);
    ~MultisampleFramebuffer();

    MultisampleFramebuffer(const MultisampleFramebuffer&) = delete;
    MultisampleFramebuffer& operator=(const MultisampleFramebuffer&) = delete;

    void Bind() const;
    void Resize(int width, int height);
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

    int m_Width = 0;
    int m_Height = 0;
    int m_Samples = 4;
};