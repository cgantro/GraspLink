#pragma once

#include <cstdint>
#include <memory>

/**
 * @brief OpenGL Texture Object의 lifetime과 texture-unit binding을 관리한다.
 *
 * @details
 * Create2D는 CPU image pixel을 GPU texture로 업로드한다. Base color 같은 색상 texture는
 * sRGB 내부 포맷으로 저장하면 sampling 시 GPU가 자동으로 linear space로 변환한다.
 * 반면 normal/roughness 같은 데이터 texture는 sRGB 변환을 사용하면 안 된다.
 *
 * @todo [FUTURE] Texture usage/semantic을 도입해 baseColor는 sRGB, normal/metallic 등은 linear로 자동 선택한다.
 * @todo [FUTURE] glTF sampler의 wrap/filter 설정을 반영한다.
 */
class Texture final
{
public:
    Texture();

    /** @brief 소유한 OpenGL texture object를 삭제한다. */
    ~Texture();

    Texture(const Texture&) = delete;
    Texture& operator=(const Texture&) = delete;

    /** @brief texture를 지정 texture unit에 바인딩한다. */
    void Bind(std::uint32_t slot = 0) const;

    /**
     * @brief 8-bit CPU pixel 데이터로 2D texture를 생성한다.
     * @param width image width.
     * @param height image height.
     * @param channels 1, 3 또는 4 channel.
     * @param pixels CPU pixel buffer 시작 주소.
     * @param srgb RGB/RGBA를 sRGB 내부 포맷으로 저장할지 여부.
     */
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
