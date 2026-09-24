#include "Texture.h"

#include <glad/glad.h>

#include <stdexcept>


// ============================================================================
// OpenGL Format Mapping
// ============================================================================

Texture::OpenGLFormatInfo Texture::GetOpenGLFormatInfo(
    TextureFormat format
)
{
    switch (format)
    {
    case TextureFormat::RGB8:
        return {
            GL_RGB8,
            GL_RGB,
            GL_UNSIGNED_BYTE
        };


    case TextureFormat::RGBA8:
        return {
            GL_RGBA8,
            GL_RGBA,
            GL_UNSIGNED_BYTE
        };


    case TextureFormat::RGBA16F:
        /*
            HDR / IBL용 floating-point Texture.

            internal:
                GPU는 각 channel을 16-bit float로 저장한다.

            upload type:
                CPU에서는 float 배열을 전달한다.
        */
        return {
            GL_RGBA16F,
            GL_RGBA,
            GL_FLOAT
        };


    case TextureFormat::DEPTH24_STENCIL8:
        return {
            GL_DEPTH24_STENCIL8,
            GL_DEPTH_STENCIL,
            GL_UNSIGNED_INT_24_8
        };
    }


    throw std::runtime_error(
        "Unsupported TextureFormat"
    );
}


// ============================================================================
// Constructor
// ============================================================================

Texture::Texture(
    int width,
    int height,
    TextureFormat format,
    TextureType type
)
    : m_Width(width),
      m_Height(height),
      m_Format(format),
      m_Type(type)
{
    if (width <= 0 || height <= 0)
    {
        throw std::invalid_argument(
            "Texture size must be greater than zero"
        );
    }


    CreateInternal();
}


Texture::Texture(
    int width,
    int height,
    const void* data,
    TextureFormat format
)
    : m_Width(width),
      m_Height(height),
      m_Format(format),
      m_Type(TextureType::Texture2D)
{
    if (width <= 0 || height <= 0)
    {
        throw std::invalid_argument(
            "Texture size must be greater than zero"
        );
    }


    CreateInternal();


    if (data)
    {
        SetData(data);
    }
}


// ============================================================================
// Destructor
// ============================================================================

Texture::~Texture()
{
    /*
        glGenTextures로 생성한 GPU Resource는
        반드시 동일한 OpenGL context가 살아 있는 동안 삭제해야 한다.

        따라서 ViewerApp 종료 순서에서도:

            AssetManager cleanup
                ↓
            OpenGL Window destruction

        순서를 유지해야 한다.
    */
    if (m_RendererID != 0)
    {
        glDeleteTextures(
            1,
            &m_RendererID
        );

        m_RendererID = 0;
    }
}


// ============================================================================
// Create
// ============================================================================

void Texture::CreateInternal()
{
    const OpenGLFormatInfo info =
        GetOpenGLFormatInfo(
            m_Format
        );


    glGenTextures(
        1,
        &m_RendererID
    );


    // ------------------------------------------------------------------------
    // Texture2D
    // ------------------------------------------------------------------------

    if (m_Type == TextureType::Texture2D)
    {
        glBindTexture(
            GL_TEXTURE_2D,
            m_RendererID
        );


        /*
            GPU Texture Storage 확보.

            마지막 nullptr은:

                "아직 pixel data는 없지만 GPU memory는 확보해라"

            라는 의미다.

            이후 SetData() 또는 Framebuffer rendering으로
            내용이 채워질 수 있다.
        */
        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            static_cast<GLint>(info.internalFormat),
            m_Width,
            m_Height,
            0,
            info.dataFormat,
            info.dataType,
            nullptr
        );


        /*
            기본 Sampling 설정.

            확대/축소 시 주변 texel을 선형 보간한다.
        */
        if (m_Format == TextureFormat::DEPTH24_STENCIL8)
        {
            /*
                Depth 값은 일반 Color Texture처럼 보간하는 것보다
                현재 단계에서는 NEAREST가 안전하다.
            */
            glTexParameteri(
                GL_TEXTURE_2D,
                GL_TEXTURE_MIN_FILTER,
                GL_NEAREST
            );

            glTexParameteri(
                GL_TEXTURE_2D,
                GL_TEXTURE_MAG_FILTER,
                GL_NEAREST
            );
        }
        else
        {
            glTexParameteri(
                GL_TEXTURE_2D,
                GL_TEXTURE_MIN_FILTER,
                GL_LINEAR
            );

            glTexParameteri(
                GL_TEXTURE_2D,
                GL_TEXTURE_MAG_FILTER,
                GL_LINEAR
            );
        }


        /*
            UV가 0~1 범위를 벗어나더라도 가장자리 값을 사용한다.

            PBR Texture에서도 안전한 기본값이고,
            BRDF LUT에서도 적절하다.
        */
        glTexParameteri(
            GL_TEXTURE_2D,
            GL_TEXTURE_WRAP_S,
            GL_CLAMP_TO_EDGE
        );

        glTexParameteri(
            GL_TEXTURE_2D,
            GL_TEXTURE_WRAP_T,
            GL_CLAMP_TO_EDGE
        );


        glBindTexture(
            GL_TEXTURE_2D,
            0
        );

        return;
    }


    // ------------------------------------------------------------------------
    // TextureCube
    // ------------------------------------------------------------------------

    glBindTexture(
        GL_TEXTURE_CUBE_MAP,
        m_RendererID
    );


    /*
        Cubemap은 실제로 6개의 2D Texture Face로 구성된다.

            +X
            -X
            +Y
            -Y
            +Z
            -Z

        하지만 OpenGL에서는 하나의 Texture Object로 관리한다.
    */
    for (int face = 0; face < 6; ++face)
    {
        glTexImage2D(
            GL_TEXTURE_CUBE_MAP_POSITIVE_X + face,
            0,
            static_cast<GLint>(info.internalFormat),
            m_Width,
            m_Height,
            0,
            info.dataFormat,
            info.dataType,
            nullptr
        );
    }


    /*
        Cubemap edge에서 반대편 face와 이어지므로
        REPEAT를 사용하면 seam이 생길 수 있다.

        IBL Cubemap은 CLAMP_TO_EDGE가 기본이다.
    */
    glTexParameteri(
        GL_TEXTURE_CUBE_MAP,
        GL_TEXTURE_WRAP_S,
        GL_CLAMP_TO_EDGE
    );

    glTexParameteri(
        GL_TEXTURE_CUBE_MAP,
        GL_TEXTURE_WRAP_T,
        GL_CLAMP_TO_EDGE
    );

    glTexParameteri(
        GL_TEXTURE_CUBE_MAP,
        GL_TEXTURE_WRAP_R,
        GL_CLAMP_TO_EDGE
    );


    /*
        아직 mipmap을 생성하지 않았으므로 기본은 LINEAR.

        IBL Prefilter Map에서 GenerateMipmaps()를 호출하면
        MIN_FILTER를 LINEAR_MIPMAP_LINEAR로 변경한다.
    */
    glTexParameteri(
        GL_TEXTURE_CUBE_MAP,
        GL_TEXTURE_MIN_FILTER,
        GL_LINEAR
    );

    glTexParameteri(
        GL_TEXTURE_CUBE_MAP,
        GL_TEXTURE_MAG_FILTER,
        GL_LINEAR
    );


    glBindTexture(
        GL_TEXTURE_CUBE_MAP,
        0
    );
}


// ============================================================================
// Binding
// ============================================================================

void Texture::Bind(
    std::uint32_t slot
) const
{
    /*
        OpenGL에서는 Shader Sampler가 Texture Object를 직접 가리키지 않는다.

        대신:

            Texture Unit 0
            Texture Unit 1
            ...

        중 하나를 사용한다.

        예:

            texture.Bind(3);

        는 Texture Unit 3에 Texture를 연결한다.
    */
    glActiveTexture(
        GL_TEXTURE0 + slot
    );


    const GLenum target =
        m_Type == TextureType::TextureCube
            ? GL_TEXTURE_CUBE_MAP
            : GL_TEXTURE_2D;


    glBindTexture(
        target,
        m_RendererID
    );
}


void Texture::Unbind() const
{
    const GLenum target =
        m_Type == TextureType::TextureCube
            ? GL_TEXTURE_CUBE_MAP
            : GL_TEXTURE_2D;


    glBindTexture(
        target,
        0
    );
}


// ============================================================================
// SetData
// ============================================================================

void Texture::SetData(
    const void* data
)
{
    if (m_Type != TextureType::Texture2D)
    {
        throw std::runtime_error(
            "SetData() is only valid for Texture2D"
        );
    }


    if (!data)
    {
        return;
    }


    const OpenGLFormatInfo info =
        GetOpenGLFormatInfo(
            m_Format
        );


    glBindTexture(
        GL_TEXTURE_2D,
        m_RendererID
    );


    /*
        RGB Texture의 경우 한 pixel이 3 byte이므로
        기본 4-byte alignment와 맞지 않을 수 있다.

        GL_UNPACK_ALIGNMENT = 1로 설정하면
        row padding 없이 연속된 pixel data를 읽는다.
    */
    glPixelStorei(
        GL_UNPACK_ALIGNMENT,
        1
    );


    glTexSubImage2D(
        GL_TEXTURE_2D,
        0,
        0,
        0,
        m_Width,
        m_Height,
        info.dataFormat,
        info.dataType,
        data
    );


    /*
        OpenGL 기본 상태인 4-byte alignment로 복구한다.

        OpenGL State는 전역 Context 상태이므로
        변경 후 복구하지 않으면 다른 Texture upload에 영향을 줄 수 있다.
    */
    glPixelStorei(
        GL_UNPACK_ALIGNMENT,
        4
    );


    glBindTexture(
        GL_TEXTURE_2D,
        0
    );
}


// ============================================================================
// Cubemap Data
// ============================================================================

void Texture::SetCubeFaceData(
    int faceIndex,
    const void* data,
    int mipLevel
)
{
    if (m_Type != TextureType::TextureCube)
    {
        throw std::runtime_error(
            "SetCubeFaceData() requires TextureCube"
        );
    }


    if (faceIndex < 0 || faceIndex >= 6)
    {
        throw std::out_of_range(
            "Cubemap face index must be between 0 and 5"
        );
    }


    if (mipLevel < 0)
    {
        throw std::invalid_argument(
            "Mip level cannot be negative"
        );
    }


    if (!data)
    {
        return;
    }


    const OpenGLFormatInfo info =
        GetOpenGLFormatInfo(
            m_Format
        );


    /*
        각 mip level은 원본보다 절반씩 작다.

            mip 0 = 512
            mip 1 = 256
            mip 2 = 128
            ...

        최소 크기는 1.
    */
    int mipWidth =
        m_Width >> mipLevel;

    int mipHeight =
        m_Height >> mipLevel;


    if (mipWidth < 1)
    {
        mipWidth = 1;
    }

    if (mipHeight < 1)
    {
        mipHeight = 1;
    }


    glBindTexture(
        GL_TEXTURE_CUBE_MAP,
        m_RendererID
    );


    glPixelStorei(
        GL_UNPACK_ALIGNMENT,
        1
    );


    glTexSubImage2D(
        GL_TEXTURE_CUBE_MAP_POSITIVE_X + faceIndex,
        mipLevel,
        0,
        0,
        mipWidth,
        mipHeight,
        info.dataFormat,
        info.dataType,
        data
    );


    glPixelStorei(
        GL_UNPACK_ALIGNMENT,
        4
    );


    glBindTexture(
        GL_TEXTURE_CUBE_MAP,
        0
    );
}


// ============================================================================
// Resize
// ============================================================================

void Texture::Resize(
    int width,
    int height
)
{
    if (width <= 0 || height <= 0)
    {
        return;
    }


    if (width == m_Width &&
        height == m_Height)
    {
        return;
    }


    m_Width = width;
    m_Height = height;


    /*
        Texture Storage의 크기는 생성 후 자동으로 변하지 않는다.

        따라서 기존 OpenGL Texture Object를 제거하고
        새로운 크기로 다시 생성한다.

        FBO Resize에서도 이 방식을 사용하게 된다.
    */
    if (m_RendererID != 0)
    {
        glDeleteTextures(
            1,
            &m_RendererID
        );

        m_RendererID = 0;
    }


    CreateInternal();
}


// ============================================================================
// Mipmap
// ============================================================================

void Texture::GenerateMipmaps()
{
    const GLenum target =
        m_Type == TextureType::TextureCube
            ? GL_TEXTURE_CUBE_MAP
            : GL_TEXTURE_2D;


    glBindTexture(
        target,
        m_RendererID
    );


    /*
        현재 mip 0의 내용을 기반으로 GPU가
        나머지 mip level을 생성한다.

        Cubemap이라면 6 face 모두에 대해 mip chain이 생성된다.
    */
    glGenerateMipmap(
        target
    );


    /*
        mipmap을 만들었으므로 축소 Sampling에서
        두 mip level 사이도 선형 보간하는 Trilinear Filtering 사용.
    */
    glTexParameteri(
        target,
        GL_TEXTURE_MIN_FILTER,
        GL_LINEAR_MIPMAP_LINEAR
    );


    glBindTexture(
        target,
        0
    );
}