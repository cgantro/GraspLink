#include "Texture.h"

#include <glad/glad.h>

#include <stdexcept>

Texture::Texture() = default;

Texture::~Texture()
{
    // GL object 삭제도 context를 요구한다. 소유자가 context 종료 뒤 살아남지 않도록 정리 순서를 맞춘다.
    if (m_RendererID != 0)
        glDeleteTextures(1, &m_RendererID);
}

void Texture::Bind(std::uint32_t slot) const
{
    // 활성 unit 변경과 target 바인딩을 함께 수행한다. sampler uniform 값은 호출자가 별도로 설정한다.
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
    // 유효하지 않은 입력을 GL 상태 변경 전에 거부해 부분 생성된 ID가 남지 않게 한다.
    if (width <= 0 || height <= 0 || pixels == nullptr)
        throw std::runtime_error("Invalid texture data");

    // format은 CPU 배열의 채널 배치, internalFormat은 GPU 저장 형식과 색 변환 방식을 정한다.
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

    // 작은 화면 크기로 축소될 때 미리 만든 mip 단계와 선형 보간을 사용한다.
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

    // 입력: 행 사이 padding 없는 CPU pixel 배열을 읽도록 기본 4-byte 정렬을 1로 변경.
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

    // 이유: 축소 이미지를 미리 만들어 멀리 있는 표면의 texture 깜빡임을 줄임.
    glGenerateMipmap(GL_TEXTURE_2D);

    glBindTexture(GL_TEXTURE_2D, 0);

    // 제한: glTF sampler 설정 대신 고정 반복·필터 규칙 사용.
    return texture;
}
