#include "Texture.h"

#include <glad/glad.h>
#include <stdexcept>

namespace PoseLink {

Texture::Texture(int width, int height)
    : m_Width(width), m_Height(height)
{
    if (width <= 0 || height <= 0)
        throw std::runtime_error(
            "Texture size must be greater than 0"
        );

    // 생성자가 끝난 시점에는
    // GPU에 이 Texture가 사용할 저장 공간까지 만들어져 있어야 한다.
    CreateInternal();
}

Texture::~Texture()
{
    /*
        glGenTextures()로 만든 Texture Object는
        OpenGL Driver가 관리하는 GPU 자원이다.

        C++ 객체가 사라진다고 GPU 자원이 자동으로 없어지는 것이 아니므로
        소멸자에서 직접 삭제한다.

        즉 Texture 클래스는 OpenGL Texture Object를 소유하는
        RAII Wrapper 역할을 한다.
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

void Texture::CreateInternal()
{
    /*
        1. Texture Object 생성

        OpenGL에게 새로운 Texture Object 하나를 만들어 달라고 요청한다.

        m_RendererID는 Texture 자체가 아니라,
        OpenGL이 해당 Texture Object를 식별하기 위해 사용하는 ID다.

        예:
            m_RendererID = 3

        이후 OpenGL 함수들은 이 ID를 통해
        어떤 Texture를 사용할 것인지 구분한다.
    */
    glGenTextures(
        1,
        &m_RendererID
    );

    /*
        2. 현재 작업 대상 Texture 설정

        OpenGL은 State Machine이므로 대부분의 함수가

            "Texture 3을 수정해"

        처럼 ID를 직접 받지 않는다.

        대신 먼저:

            glBindTexture(..., 3)

        으로 현재 GL_TEXTURE_2D 작업 대상으로 지정한 뒤,

            glTexImage2D()
            glTexParameteri()

        같은 함수를 호출한다.

        아래 호출부터는 m_RendererID Texture에 설정이 적용된다.
    */
    glBindTexture(
        GL_TEXTURE_2D,
        m_RendererID
    );

    /*
        3. GPU에 Texture 저장 공간 생성

        2D Texture의 크기와 Pixel Format을 정의하고
        GPU가 사용할 메모리 공간을 만든다.

        현재 설정:

            Width  = m_Width
            Height = m_Height

            한 Pixel = RGBA
                      = 4 Channels
                      = 각 Channel 8bit
                      = 총 4Byte

        GL_RGBA8
            → GPU 내부에서 Texture를 어떤 형식으로 저장할지

        GL_RGBA
            → CPU에서 전달되는 Pixel의 Channel 순서

        GL_UNSIGNED_BYTE
            → 각 Channel 하나가 unsigned char(0~255)라는 의미

        마지막 인자가 nullptr이므로
        지금은 Pixel 데이터는 전달하지 않고 저장 공간만 만든다.

        실제 Pixel 데이터는 Update()에서
        glTexSubImage2D()를 이용해 넣는다.
    */
    glTexImage2D(
        GL_TEXTURE_2D,
        0,                  // Mipmap Level. 현재는 Mipmap을 사용하지 않으므로 0
        GL_RGBA8,           // GPU 내부 저장 형식
        m_Width,
        m_Height,
        0,                  // Legacy border 값. 항상 0 사용
        GL_RGBA,            // CPU Pixel 데이터의 Channel 순서
        GL_UNSIGNED_BYTE,   // Channel 하나의 데이터 타입
        nullptr             // 아직 Pixel 데이터는 올리지 않음
    );

    /*
        4. Texture Sampling 방법 설정

        Texture의 Pixel 수와 실제 화면에 그려지는 Pixel 수가
        항상 같지는 않다.

        예를 들어 100x100 Texture를
        화면에 500x500으로 확대하면
        하나의 Texture Pixel을 여러 화면 Pixel이 사용하게 된다.

        이때 "어떤 Pixel 값을 사용할지" 정하는 것이 Texture Filter다.
    */

    /*
        MIN_FILTER:
        Texture가 화면에서 원래 크기보다 작게 보일 때 사용한다.

        GL_NEAREST는 가장 가까운 Pixel 하나를 그대로 선택한다.

        보간하지 않기 때문에 Pixel 경계가 명확하게 보인다.
    */
    glTexParameteri(
        GL_TEXTURE_2D,
        GL_TEXTURE_MIN_FILTER,
        GL_NEAREST
    );

    /*
        MAG_FILTER:
        Texture가 화면에서 원래 크기보다 크게 보일 때 사용한다.

        이것도 GL_NEAREST를 사용하므로
        확대했을 때 Pixel이 뚜렷하게 보인다.
    */
    glTexParameteri(
        GL_TEXTURE_2D,
        GL_TEXTURE_MAG_FILTER,
        GL_NEAREST
    );

    /*
        5. UV 좌표가 0~1 범위를 벗어날 때 처리 방식

        일반적인 Texture Coordinate 범위:

            (0,0) ---------------- (1,0)
              |                      |
              |       Texture        |
              |                      |
            (0,1) ---------------- (1,1)

        그런데 Shader에서 UV가 1.2, -0.1처럼 범위를 벗어날 수도 있다.

        GL_CLAMP_TO_EDGE를 사용하면
        범위를 벗어난 위치에서 가장 가까운 가장자리 Pixel을 사용한다.

        PoseLink에서는 영상/이미지를 표시할 예정이므로
        Texture를 반복시키는 GL_REPEAT보다 이 방식이 자연스럽다.
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

    /*
        설정이 끝났으므로 현재 GL_TEXTURE_2D Binding을 해제한다.

        Texture 자체가 삭제되는 것은 아니다.

        단순히:

            "현재 작업 중인 GL_TEXTURE_2D Texture 없음"

        상태로 변경하는 것이다.
    */
    glBindTexture(
        GL_TEXTURE_2D,
        0
    );
}

void Texture::Update(const void* data)
{
    if (data == nullptr)
        return;

    /*
        glTexSubImage2D()는 현재 Bind되어 있는 Texture를 수정한다.

        따라서 먼저 이 Texture를 현재 작업 대상으로 설정해야 한다.
    */
    glBindTexture(
        GL_TEXTURE_2D,
        m_RendererID
    );

    /*
        CPU Pixel 데이터의 Row 정렬 방식 설정.

        OpenGL의 기본 GL_UNPACK_ALIGNMENT는 4다.

        즉 OpenGL은 CPU 메모리에서 다음 Pixel Row가
        4Byte 경계에 맞춰 시작한다고 기본적으로 가정한다.

        RGBA8의 경우:

            Pixel 하나 = 4Byte

        이므로 대부분 문제가 없다.

        하지만 나중에 RGB/BGR 영상처럼

            Pixel 하나 = 3Byte

        인 데이터를 사용할 경우,
        한 Row 크기가 4의 배수가 아닐 수 있다.

        예:
            width = 5
            RGB = 3Byte

            한 Row = 15Byte

        기본 alignment=4로 읽으면 OpenGL이
        다음 Row가 16Byte 위치에서 시작한다고 판단할 수 있어
        영상이 밀려 보일 수 있다.

        그래서 입력 Pixel 데이터를 1Byte 단위로
        연속해서 읽도록 설정한다.
    */

    GLint previousAlignment = 0;

    // 다른 코드가 설정해 둔 기존 Alignment 값을 보존한다.
    glGetIntegerv(
        GL_UNPACK_ALIGNMENT,
        &previousAlignment
    );

    glPixelStorei(
        GL_UNPACK_ALIGNMENT,
        1
    );

    /*
        이미 만들어 놓은 Texture 저장 공간의 Pixel 내용만 교체한다.

        glTexImage2D()
            → Texture 크기/Format/저장 공간 자체를 정의

        glTexSubImage2D()
            → 이미 존재하는 저장 공간의 Pixel 데이터만 갱신

        영상처럼 매 Frame 같은 크기의 Pixel 데이터가 들어오는 경우
        저장 공간을 매번 다시 만드는 것보다
        glTexSubImage2D()로 내용만 바꾸는 것이 적절하다.
    */
    glTexSubImage2D(
        GL_TEXTURE_2D,
        0,                  // Mipmap Level
        0,                  // 갱신 시작 X
        0,                  // 갱신 시작 Y
        m_Width,
        m_Height,
        GL_RGBA,            // CPU 데이터의 Channel 순서
        GL_UNSIGNED_BYTE,   // Channel 하나의 타입
        data                // CPU Pixel 데이터 시작 주소
    );

    /*
        GL_UNPACK_ALIGNMENT도 OpenGL Context의 전역 상태이므로
        Texture Update 이후 원래 값으로 복원한다.

        단순히 4로 고정하는 것보다
        이전 상태를 저장했다가 복원하는 편이 안전하다.
    */
    glPixelStorei(
        GL_UNPACK_ALIGNMENT,
        previousAlignment
    );

    glBindTexture(
        GL_TEXTURE_2D,
        0
    );
}

void Texture::Bind(uint32_t slot) const
{
    /*
        Texture를 Shader에서 사용하려면
        Texture Object를 바로 Shader에 전달하는 것이 아니다.

        OpenGL은 중간에 "Texture Unit"이라는 슬롯을 사용한다.

        구조:

            Shader sampler2D
                    │
                    │ 0
                    ▼
            Texture Unit 0
                    │
                    ▼
            Texture Object

        예를 들어 GLSL에:

            uniform sampler2D u_Texture;

        가 있고 CPU에서:

            shader.SetInt("u_Texture", 0);

        을 호출하면:

            u_Texture는 Texture Unit 0을 읽어라

        라는 뜻이다.

        따라서 여기서는 먼저 slot에 해당하는
        Texture Unit을 활성화한다.
    */

    glActiveTexture(
        GL_TEXTURE0 + slot
    );

    /*
        방금 선택한 Texture Unit에
        현재 Texture Object를 연결한다.

        예:

            slot = 0

            Texture Unit 0
                  │
                  ▼
            m_RendererID

        이제 Shader의 sampler가 0번 Unit을 바라보고 있다면
        이 Texture의 Pixel을 읽을 수 있다.
    */
    glBindTexture(
        GL_TEXTURE_2D,
        m_RendererID
    );
}

void Texture::UnBind() const
{
    /*
        현재 활성화되어 있는 Texture Unit에서
        GL_TEXTURE_2D Binding을 제거한다.

        Texture Object 자체를 삭제하는 것은 아니다.

        단순히 현재 OpenGL Context의 Binding 상태만:

            Texture Unit
                │
                X

        로 변경한다.
    */
    glBindTexture(
        GL_TEXTURE_2D,
        0
    );
}

} // namespace PoseLink