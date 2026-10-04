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
/*
 * [추가 그래픽스 용어 설명]
 * - Texture: 2D 이미지나 수치 데이터를 GPU에서 sampling할 수 있게 저장한 리소스.
 * - Sampling: UV 좌표를 이용해 texture의 특정 위치 값을 읽는 과정.
 * - Texture Unit: Shader가 여러 texture를 동시에 읽을 때 각 texture를 연결하는 slot.
 * - sRGB: 사람이 보는 밝기 특성에 맞춘 비선형 색 공간. 색상 texture에 사용한다.
 * - Linear Space: 조명 계산을 수행하기 적합한 선형 색 공간.
 * - Channel: pixel 하나가 가진 성분 수. 예: RGB=3, RGBA=4.
 * - Wrap/Filter: UV가 범위를 벗어나거나 pixel보다 확대/축소될 때 texture를 읽는 규칙.
 *
 * width/height는 pixel 단위, channels는 pixel당 성분 개수다.
 * pixels는 CPU 메모리의 8-bit raw image buffer 시작 주소다.
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
    // OpenGL이 발급한 texture object ID. 0은 아직 생성되지 않은 초기 상태.
    std::uint32_t m_RendererID = 0;

    // OpenGL texture target enum. 현재 2D texture이면 GL_TEXTURE_2D 계열 값이 들어간다.
    std::uint32_t m_Target = 0;
};
