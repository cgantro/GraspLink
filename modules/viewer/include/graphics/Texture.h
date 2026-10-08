#pragma once

#include <cstdint>
#include <memory>

namespace grasplink::graphics
{

/**
 * @brief Texture는 GPU가 표면 색을 찾을 때 읽는 이미지이며 이 클래스는 8 bit 채널의 2D 이미지를 만든다.
 * @details
 * Create2D는 픽셀을 GPU로 복사한 뒤 호출자가 준 CPU 주소를 보관하지 않는다. Material은 기본색 Texture를 공유한다.
 * Renderer는 기본색 이미지를 0번 Texture unit에서 읽는다.
 * 마지막 공유 참조가 사라지면 이 객체가 GPU 이미지 번호를 삭제한다. 만들거나 삭제할 때는 GPU 명령 실행 환경인 OpenGL context가 현재 스레드에서 활성화되어야 한다.
 * 크기·채널·포인터 오류는 std::runtime_error로 알리지만, OpenGL 실행 환경이 없거나 GPU 오류가 난 경우는 예외로 바꾸지 않는다.
 */
class Texture final
{
public:
    Texture();

    ~Texture();

    Texture(const Texture&) = delete;
    Texture& operator=(const Texture&) = delete;

    /**
     * @brief GPU의 Texture unit(이미지 연결 슬롯)을 선택하고 이 Texture를 그 슬롯에 연결한다.
     * @param slot OpenGL 이미지 슬롯 번호. Shader의 이미지 입력에도 이 번호를 전달해야 GPU가 연결된 이미지를 읽는다.
     */
    void Bind(std::uint32_t slot = 0) const;

    /**
     * @brief 픽셀마다 8-bit 채널로 연속 저장한 CPU 이미지 배열을 GPU Texture로 만든다.
     * @param width 이미지 너비 [pixel], 양수.
     * @param height 이미지 높이 [pixel], 양수.
     * @param channels 픽셀마다 있는 채널 수. 1(밝기), 3(RGB 색), 4(RGBA 색과 투명도)만 지원한다.
     * @param pixels 최소 width*height*channels byte가 연속된 배열. 함수가 호출된 동안 복사하며 주소는 보관하지 않는다.
     * @param srgb RGB가 화면 색이면 true. GPU가 색 채널을 빛 계산용 선형 값으로 바꾸어 읽는다.
     * alpha는 바꾸지 않으며 한 채널 이미지는 항상 R8로 저장한다.
     * @return GPU 이미지를 함께 소유하는 공유 참조.
     * @throws std::runtime_error 크기/포인터가 유효하지 않거나 channels가 1, 3, 4가 아닐 때.
     * @details 화면에서 이미지가 작아지면 미리 축소해 둔 단계별 이미지를 골라 부드럽게 섞으며 커질 때도 이웃 픽셀을 섞는다.
     * 이미지는 가로·세로 방향으로 반복하고 입력 배열의 첫 행부터 그대로 복사하므로 위아래를 뒤집지 않는다.
     * glTF 기본색 이미지는 TinyGLTF가 디코드한 배열을 그대로 사용한다. 이 Viewer는 금속성·거칠기 숫자 이미지를 재질에 연결하지 않는다.
     * 파일별 이미지 필터와 반복 규칙은 반영하지 않는다.
     */
    static std::shared_ptr<Texture> Create2D(
        int width,
        int height,
        int channels,
        const unsigned char* pixels,
        bool srgb = true);

private:
    // GPU가 부여한 이미지 번호. 0은 아직 만들지 않은 상태다.
    std::uint32_t m_RendererID = 0;

    // 이미지 모양. 현재는 평면 2D 이미지만 만든다.
    std::uint32_t m_Target = 0;
};

} // namespace grasplink::graphics
