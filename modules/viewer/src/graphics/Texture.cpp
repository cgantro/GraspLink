#include "Texture.h"

#include <glad/glad.h>

#include <stdexcept>

Texture::Texture() = default;

Texture::~Texture()
{
    if (m_RendererID != 0)
        glDeleteTextures(1, &m_RendererID);
}

void Texture::Bind(std::uint32_t slot) const
{
    glActiveTexture(GL_TEXTURE0 + slot);
    glBindTexture(m_Target, m_RendererID);
}

std::shared_ptr<Texture> Texture::Create2D(
    int width,
    int height,
    int channels,
    const unsigned char* pixels,
    bool srgb)
{
    if (width <= 0 || height <= 0 || pixels == nullptr)
        throw std::runtime_error("Invalid texture data");

    GLenum format = GL_RGB;
    GLenum internalFormat = GL_RGB8;

    if (channels == 1)
    {
        format = GL_RED;
        internalFormat = GL_R8;
    }
    else if (channels == 3)
    {
        format = GL_RGB;
        internalFormat = srgb ? GL_SRGB8 : GL_RGB8;
    }
    else if (channels == 4)
    {
        format = GL_RGBA;
        internalFormat = srgb ? GL_SRGB8_ALPHA8 : GL_RGBA8;
    }
    else
    {
        throw std::runtime_error("Unsupported texture channel count");
    }

    auto texture = std::make_shared<Texture>();
    texture->m_Target = GL_TEXTURE_2D;

    glGenTextures(1, &texture->m_RendererID);
    glBindTexture(GL_TEXTURE_2D, texture->m_RendererID);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    glTexImage2D(
        GL_TEXTURE_2D,
        0,
        internalFormat,
        width,
        height,
        0,
        format,
        GL_UNSIGNED_BYTE,
        pixels);

    glGenerateMipmap(GL_TEXTURE_2D);

    glBindTexture(GL_TEXTURE_2D, 0);

    return texture;
}