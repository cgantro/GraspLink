#pragma once

#include <cstdint>
#include <memory>

/**
 * @brief 8-bit CPU 이미지를 OpenGL 2D texture로 올려 shader에서 읽게 한다.
 * @details
 * Create2D는 전달받은 픽셀을 업로드한 뒤 CPU 포인터를 보관하지 않는다. Material이 base color
 * texture를 공유하고 Renderer는 이를 unit 0에 묶는다. ShadowMap은 별도 unit 7을 사용한다.
 * 이 클래스는 GL texture ID만 소유하며 마지막 shared_ptr가 해제될 때 glDeleteTextures를 호출한다.
 * 생성과 파괴 모두 유효한 GL context가 현재 thread에 있어야 한다. 생성 인자 검증 실패는
 * std::runtime_error를 던지며, context 부재나 OpenGL 오류를 예외로 변환하지는 않는다.
 */
class Texture final
{
public:
    Texture();

    ~Texture();

    Texture(const Texture&) = delete;
    Texture& operator=(const Texture&) = delete;

    /**
     * @brief 이 texture를 지정한 texture unit에 바인딩한다.
     * @param slot OpenGL texture unit 번호. 연결할 sampler uniform 값과 같아야 한다.
     */
    void Bind(std::uint32_t slot = 0) const;

    /**
     * @brief 연속된 8-bit 픽셀 배열로 2D texture를 생성한다.
     * @param width 이미지 너비 [pixel], 양수.
     * @param height 이미지 높이 [pixel], 양수.
     * @param channels 성분 수. 1(RED), 3(RGB), 4(RGBA)만 지원한다.
     * @param pixels 최소 width*height*channels byte를 가진 연속 배열. 호출 중 업로드하며 포인터는 보관하지 않는다.
     * @param srgb RGB 색 texture이면 true. RGB 성분을 shader에서 선형 공간으로 읽도록 sRGB 내부 형식을 쓴다.
     *             alpha는 변환되지 않으며, 1-channel 형식은 항상 R8이다.
     * @return GPU texture를 소유하는 공유 포인터.
     * @throws std::runtime_error 크기/포인터가 유효하지 않거나 channels가 1, 3, 4가 아닐 때.
     * @details minification은 선형 mip 선택과 선형 보간, magnification은 선형 보간을 사용한다.
     *          S/T는 반복하고 mipmap을 생성한다. 입력 행 순서는 그대로 올리며 수직 반전하지 않는다.
     *          glTF 이미지는 TinyGLTF가 디코딩한 배열을 그대로 전달하므로 별도의 STB flip 설정은 없다.
     *          Metal-roughness 등의 수치 texture에는 srgb=false가 필요하지만 현재 Material/Renderer는
     *          base color texture만 사용한다. Sampler별 glTF 필터·wrap 설정은 적용하지 않는다.
     */
    static std::shared_ptr<Texture> Create2D(
        int width,
        int height,
        int channels,
        const unsigned char* pixels,
        bool srgb = true);

private:
    // 소유: GPU texture ID. 0은 미생성 상태.
    std::uint32_t m_RendererID = 0;

    // 종류: 현재는 GL_TEXTURE_2D만 생성.
    std::uint32_t m_Target = 0;
};
