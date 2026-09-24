#pragma once

#include <cstdint>


/*
    ============================================================================
    Texture
    ============================================================================

    OpenGL Texture Object를 감싸는 GPU Resource 클래스.

    현재 프로젝트에서 Texture는 크게 두 용도로 사용된다.

    1. 일반 2D Texture
       - GLB Material의 BaseColor / Normal / MetallicRoughness
       - HDR Equirectangular Texture
       - BRDF LUT

    2. CubeMap
       - Environment Map
       - IBL Prefilter Map


    AssetManager가 실제 Texture 객체의 lifetime을 관리하고,
    ECS Component에서는 ResourceID만 보관하는 구조로 사용할 예정이다.

        MeshRenderer
            ↓ ResourceID
        AssetManager           
            ↓
        Texture
            ↓
        OpenGL Texture Object
*/


// ============================================================================
// Texture Format
// ============================================================================

enum class TextureFormat
{
    /*
        일반 8-bit RGB.
        예:
            JPG Texture
    */
    RGB8,
    /*
        일반 8-bit RGBA.
        예:
            PNG
            glTF BaseColor Texture
    */
    RGBA8,
    /*
        16-bit floating point RGBA.

        HDR / IBL 계산에 사용한다.

        예:
            HDR Environment
            Prefiltered Cubemap
            BRDF LUT
    */
    RGBA16F,
    /*
        Depth + Stencil Texture.

        이번에는 Framebuffer에서 사용할 준비만 해둔다.
    */
    DEPTH24_STENCIL8
};


// ============================================================================
// Texture Type
// ============================================================================
enum class TextureType
{
    Texture2D,
    TextureCube
};
// ============================================================================
// Texture
// ============================================================================
class Texture
{
public:
    /*
        빈 GPU Texture 생성.

        Framebuffer Attachment나 Cubemap처럼
        먼저 GPU storage만 확보할 때 사용한다.

        예:

            Texture(
                512,
                512,
                TextureFormat::RGBA16F,
                TextureType::TextureCube
            );
    */
    Texture(
        int width,
        int height,
        TextureFormat format = TextureFormat::RGBA8,
        TextureType type = TextureType::Texture2D
    );

    /*
        CPU Image Data를 바로 GPU로 업로드한다.

        이 생성자는 Texture2D 용도다.

        data의 실제 타입은 format에 따라 달라진다.

            RGB8 / RGBA8
                -> uint8_t*

            RGBA16F
                -> float*
    */
    Texture(
        int width,
        int height,
        const void* data,
        TextureFormat format = TextureFormat::RGBA8
    );
    ~Texture();
    /*
        OpenGL Resource는 복사하면 하나의 Renderer ID를
        여러 객체가 소유하게 되는 문제가 발생한다.

        따라서 복사를 금지한다.
    */
    Texture(const Texture&) = delete;
    Texture& operator=(const Texture&) = delete;
    // ------------------------------------------------------------------------
    // Binding
    // ------------------------------------------------------------------------

    /*
        Texture Unit에 바인딩한다.

        slot = 0이면:

            GL_TEXTURE0

        slot = 1이면:

            GL_TEXTURE1
    */
    void Bind(std::uint32_t slot = 0) const;
    void Unbind() const;
    // ------------------------------------------------------------------------
    // Data
    // ------------------------------------------------------------------------

    /*
        기존 Texture2D GPU storage에 데이터를 업로드한다.

        TextureCube에서는 사용하지 않는다.
    */
    void SetData(const void* data);
    /*
        Cubemap의 특정 face에 CPU 데이터를 업로드한다.

        faceIndex:

            0 = +X
            1 = -X
            2 = +Y
            3 = -Y
            4 = +Z
            5 = -Z
    */
    void SetCubeFaceData(
        int faceIndex,
        const void* data,
        int mipLevel = 0
    );
    /*
        GPU Texture storage를 새로운 크기로 다시 만든다.

        Framebuffer resize 등에 사용한다.
    */
    void Resize(
        int width,
        int height
    );
    /*
        현재 Texture의 mip chain을 GPU에서 생성한다.

        IBL Prefilter Cubemap에서 필요하다.
    */
    void GenerateMipmaps();
    // ------------------------------------------------------------------------
    // Getters
    // ------------------------------------------------------------------------
    std::uint32_t GetID() const noexcept
    {
        return m_RendererID;
    }

    int GetWidth() const noexcept
    {
        return m_Width;
    }

    int GetHeight() const noexcept
    {
        return m_Height;
    }

    TextureFormat GetFormat() const noexcept
    {
        return m_Format;
    }

    TextureType GetType() const noexcept
    {
        return m_Type;
    }


private:
    /*
        실제 OpenGL Texture Object 생성 및 Storage 할당.

        Constructor와 Resize가 공통으로 사용한다.
    */
    void CreateInternal();
    /*
        현재 format에 대응하는 OpenGL Format 정보를 반환하기 위한
        내부 구조체.

        OpenGL Texture에는 보통 세 가지 정보가 필요하다.

        internalFormat:
            GPU 내부에서 어떻게 저장할 것인가?

        dataFormat:
            CPU 데이터가 어떤 channel 구조인가?

        dataType:
            각 channel의 자료형은 무엇인가?
    */
    struct OpenGLFormatInfo
    {
        std::uint32_t internalFormat = 0;
        std::uint32_t dataFormat = 0;
        std::uint32_t dataType = 0;
    };
    static OpenGLFormatInfo GetOpenGLFormatInfo(
        TextureFormat format
    );
private:
    std::uint32_t m_RendererID = 0;

    int m_Width = 0;
    int m_Height = 0;

    TextureFormat m_Format = TextureFormat::RGBA8;
    TextureType m_Type = TextureType::Texture2D;
};