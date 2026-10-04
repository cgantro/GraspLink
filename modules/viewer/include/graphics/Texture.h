#pragma once

#include <cstdint>
#include <memory>

/**
 * @brief OpenGL Texture Object의 lifetime과 texture-unit binding을 관리한다.
 *
 * @details
 * Create2D()는 CPU 8-bit image pixel을 GPU texture로 업로드한다. Base-color처럼 색을 나타내는 texture는
 * sRGB 내부 포맷을 사용하면 sampling 시 GPU가 linear space로 변환할 수 있다. 반면 normal/roughness 같은
 * 데이터 texture에는 sRGB 변환을 적용하면 안 된다.
 *
 * width/height는 pixel 수, channels는 pixel당 component 수이며 world-space meter 단위와 무관하다.
 *
 * @todo [FUTURE] Texture semantic을 도입해 baseColor는 sRGB, normal/metallic 등은 linear로 자동 선택한다.
 * @todo [FUTURE] glTF sampler의 wrap/filter 설정을 반영한다.
 */
class Texture final
{
public:
    /** @brief 아직 GPU storage가 없는 Texture wrapper를 생성한다. */
    Texture();

    /** @brief 소유한 OpenGL texture object를 삭제한다. */
    ~Texture();

    /** @brief OpenGL texture ownership 중복을 막기 위해 copy construction을 금지한다. */
    Texture(const Texture&) = delete;

    /** @brief OpenGL texture ownership 중복을 막기 위해 copy assignment를 금지한다. */
    Texture& operator=(const Texture&) = delete;

    /**
     * @brief 이 texture를 지정 texture unit에 바인딩한다.
     * @param slot OpenGL texture unit index. 기본 0.
     */
    void Bind(std::uint32_t slot = 0) const;

    /**
     * @brief 8-bit CPU pixel 데이터로 2D texture를 생성한다.
     * @param width image width [pixel].
     * @param height image height [pixel].
     * @param channels pixel당 channel 개수. 현재 1, 3 또는 4를 처리할 수 있도록 구현 범위를 확인해야 한다.
     * @param pixels CPU pixel buffer 시작 주소. 함수 호출 시 읽기 가능한 연속 byte 영역이어야 한다.
     * @param srgb true이면 RGB/RGBA 색상 데이터를 sRGB 내부 포맷으로 저장하도록 요청한다.
     * @return GPU texture를 소유하는 shared_ptr<Texture>.
     */
    static std::shared_ptr<Texture> Create2D(
        int width,
        int height,
        int channels,
        const unsigned char* pixels,
        bool srgb = true);

private:
    /** @brief OpenGL texture object ID. 0은 생성 전/해제 후 상태. */
    std::uint32_t m_RendererID = 0;

    /** @brief OpenGL texture target enum 값. 현재 2D texture 생성 경로에서 설정된다. */
    std::uint32_t m_Target = 0;
};
