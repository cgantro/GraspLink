#include "poselink/graphics/Texture.h"
#include <glad/glad.h>
#include <stdexcept>

Texture::Texture(int width, int height):m_Width(width), m_Height(height){
    if(width <= 0 || height <= 0) throw std::runtime_error("Texture Size must be Greater than 0");
    CreateInternal();
}

Texture::~Texture(){
    if(m_RendererID != 0) {
        glDeleteTextures(1,&m_RendererID);
        m_RendererID = 0;
    }
}
void Texture::CreateInternal(){
    // 1. Texture Object 생성 해야함.
    glGenTextures(1,&m_RendererID);

    // 2. 2D Texture로 바인드
    glBindTexture(GL_TEXTURE_2D,m_RendererID);

    // 3. GPU 텍스처 저장공간 확보
    // 마지막 인자가 nullptr -> 실제 pixel 데이터를 올리는게 아닌, GPU에 공간만 만든다.
    glTexImage2D(
        GL_TEXTURE_2D,
        0, // Mipmap Level
        GL_RGBA8, // RGBA 각각 1Byte -> 총 4Byte
        m_Width,
        m_Height,
        0,
        GL_RGBA,
        GL_UNSIGNED_BYTE, 
        // 각 Channel이 unsigned char,
        // 즉 0~255 범위라는 뜻.
        nullptr
    );

    // 4. Texture 축소 필터
    // Texture 크기보다 화면에서 작게 출력될 때, 어떤 방식으로 pixel을 샘플링할지 지정한다.

    // GL_LINEAR -> Pixel을 선형 보간한다.
    // GL_NEAREST 끊는다.
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
    // 크게 출력될 때의 필터
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);

    // 5. Texture 범위를 벗어났을 때
    // UV의 정상 범위 -> 0.0 ~ 1.0
    // GL_CLAMP_TO_EDGE는 범위를 벗어나면 가장자리 pixel값을 사용한다
    // 영상은 타일처럼 반복할 이유가 없다 -> GL_REPEAT보다 이쪽이 적절하다.

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
}

void Texture::Update(const void* data){
    if(data == nullptr) return;
    // 현재 Texture를 수정대상으로 Bind
    glBindTexture(GL_TEXTURE_2D, m_RendererID);

    // Pixel 메모리 정렬
    /*
        OpenGL은 기본적으로 Pixel row가 4byte 단위로 정려되어있다고 가정한다.
        RGBA8은 한 픽셀이 4byte라 괜찮지만, 나중에 RGB/BGR 영상을 다룰 때
        4의 배수가 안 되어있을 수 있다.

        예상치 못한 데이터 밀림 방지를 위해 1byte alignment로 지정한다.
    */

    glPixelStorei(GL_UNPACK_ALIGNMENT,1);

    /*
        기존 Texture의 Pixel 데이터 갱신

        glTexImage2D()와의 차이 
        glTexSubImage2D() -> 이미 만들어진 저장 공간의 내용만 변경

        Pixel만 교체 
    */

    glTexSubImage2D(
        GL_TEXTURE_2D,
        0, // Mipmap
        0,
        0, // Texture 내부에서 갱신을 시작할 x/y 위치(전체를 바꾸므로 0,0)

        m_Width,
        m_Height,

        GL_RGBA, // CPU 데이터의 Channel 순서

        GL_UNSIGNED_BYTE, // Channel 타입

        data // CPU 픽셀 데이터 시작 주소
    );

    /*
    * 다른 코드에 영향을 주지 않도록
    * OpenGL 기본 Alignment인 4로 복구.
    */
    glPixelStorei(
        GL_UNPACK_ALIGNMENT,
        4
    );
}

void Texture::Bind(uint32_t slot) const{
    /*
        Texture Unit 선택
        Shader가 Texture 객체를 직접 가리키지 않는다.
        
        Shader Sampler
            |
        Texture Unit
            |
        Texture Object

        slot = 0 -> GL_TEXTURE0을 활성화
    */

   glActiveTexture(
        GL_TEXTURE0 + slot
   );

   // Texture Object에 현재 Texture Object 연결
   glBindTexture(GL_TEXTURE_2D,m_RendererID);
}

void Texture::UnBind() const
{
    glBindTexture(
        GL_TEXTURE_2D,
        0
    );
}
