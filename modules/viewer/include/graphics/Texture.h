#pragma once

#include <cstdint>
#include <memory>
#include <string>

class Texture final
{
public:
    Texture();
    ~Texture();

    Texture(const Texture&) = delete;
    Texture& operator=(const Texture&) = delete;

    void Bind(std::uint32_t slot = 0) const;

    static std::shared_ptr<Texture> Create2D(
        int width,
        int height,
        int channels,
        const unsigned char* pixels,
        bool srgb = true);

private:
    std::uint32_t m_RendererID = 0;
    std::uint32_t m_Target = 0;
};