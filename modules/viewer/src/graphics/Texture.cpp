#include "Texture.h"

#include <glad/glad.h>

#include <stdexcept>

Texture::Texture() = default;

Texture::~Texture()
{
    // OpenGL object ID는 Context가 살아 있는 동안 한 번만 삭제해야 한다.
    if (m_RendererID != 0)
        glDeleteTextures(1, &m_RendererID);
}

void Texture::Bind(std::uint32_t slot) const
{
    /*
        Shader의 sampler2D는 texture object 자체를 직접 가리키지 않고 texture unit 번호를 사용한다.
        예: sampler uniform=0이면 GL_TEXTURE0에 현재 바인딩된 texture를 sampling한다.
    */
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

    /*
        sRGB internal format을 사용하면 GPU가 sampling할 때 sRGB encoded color를 linear space로 변환한다.
        조명 계산은 linear space에서 해야 하므로 baseColor texture에 적합하다.
        normal/roughness 같은 데이터 texture에는 sRGB 변환을 적용하면 안 된다.
    */
    auto texture = std::make_shared<Texture>();
    texture->m_Target = GL_TEXTURE_2D;

    glGenTextures(1, &texture->m_RendererID);
    glBindTexture(GL_TEXTURE_2D, texture->m_RendererID);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    // RGB row byte 수가 4의 배수가 아닐 수 있으므로 OpenGL 기본 alignment(4)를 1로 낮춘다.
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

    // 멀리 있는 texture가 aliasing되는 것을 줄이기 위해 축소 해상도 chain을 만든다.
    glGenerateMipmap(GL_TEXTURE_2D);

    glBindTexture(GL_TEXTURE_2D, 0);

    // TODO(FUTURE): glTF sampler의 wrap/filter 설정을 입력으로 받아 하드코딩을 제거한다.
    return texture;
}
