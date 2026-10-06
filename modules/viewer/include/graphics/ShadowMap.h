#pragma once

#include <cstdint>

/**
 * @brief 광원에서 본 표면의 투영 깊이를 이미지로 저장해 그림자 여부를 판단한다.
 * @details
 * 투영 깊이는 광원 위치에서 표면까지의 실제 직선거리가 아니라 광원 카메라가 정한 방향으로 투영한 뒤 앞뒤 순서를 나타내는 값이다.
 * Framebuffer는 GPU가 색이나 깊이를 기록할 출력 저장소이며 화면 창 자체가 아니다. 깊이 전용 framebuffer에는 색 이미지가 연결되지 않는다.
 * GL_DEPTH_COMPONENT24는 각 깊이값을 24 bit로 저장한다. 화면을 그릴 때 같은 광원 기준 투영 깊이를 구해 이미지 값과 비교한다.
 * PCF는 주변 깊이 표본을 여러 번 비교해 가려진 비율을 평균하는 방법이다. 현재 Shader는 5×5 픽셀을 비교해 그림자 경계를 부드럽게 한다.
 * 수치 정밀도 오차 때문에 표면이 자기 자신을 가린 것처럼 생기는 얼룩을 줄이려고 작은 bias를 더한다. 너무 크게 보정하면 그림자가 물체에서 떠 보인다.
 * 이 클래스는 이미지 크기와 GPU 객체 수명, 그림자 기록 중의 출력 저장소와 화면 픽셀 영역 변경을 관리한다.
 * OpenGL context는 GPU 명령을 실행할 자원 연결과 상태를 제공하는 환경이다. 생성·삭제·호출 시 현재 스레드에서 활성화되어 있어야 하며 단지 창이 존재하는 것으로는 충분하지 않다.
 * size는 정사각형 깊이 이미지 한 변의 픽셀 수다. 기본 2048은 2048×2048 이미지다. 이미지 바깥 깊이는 1로 취급해 해당 표면이 가짜 그림자를 만들지 않게 한다.
 * @todo [FUTURE] 해상도와 깊이 보정 값을 화면 설정에서 조정할 수 있게 한다.
 */
class ShadowMap final
{
public:
    // 크기는 깊이 이미지 한 변의 픽셀 수이며 0보다 커야 한다.
    /**
     * @brief 정사각형 깊이 이미지와 여기에 깊이를 기록할 GPU 대상을 만든다.
     * @param size 한 변의 해상도 [pixel]. 양수여야 한다.
     * @throws std::invalid_argument size가 0 이하인 경우.
     * @throws std::runtime_error 깊이 이미지를 GPU 대상에 연결할 수 없는 경우.
     * @details GPU 객체를 만들므로 현재 스레드에서 OpenGL 실행 환경(context)이 활성화되어야 한다.
     */
    explicit ShadowMap(int size = 2048);

    /** @brief GPU 깊이 이미지와 기록 대상을 해제한다. 현재 스레드에서 OpenGL 실행 환경(context)이 활성화되어야 한다. */
    ~ShadowMap();

    ShadowMap(const ShadowMap&) = delete;
    ShadowMap& operator=(const ShadowMap&) = delete;

    // 변경: 깊이 기록 대상과 화면 영역을 그림자 이미지에 맞추고 이전 깊이를 지운다.
    /**
     * @brief 광원에서 본 깊이 이미지에 물체를 그리기 시작한다.
     * @details 현재 기록 대상과 그릴 픽셀 영역을 저장한 뒤 이 객체의 깊이 이미지로 바꾸고 기존 깊이를 지운다.
     * 작은 깊이 보정(offset)을 켜 같은 표면끼리의 수치 오차가 점무늬 그림자가 되는 일을 줄인다.
     * 깊이를 기록하는 기능이 이미 켜져 있어야 한다. 호출자는 이 함수와 End 사이에 광원 시점 변환으로 물체를 그린다.
     */
    void Begin();

    // 복원: Begin 전에 사용하던 기록 대상과 그릴 픽셀 영역. 이 메서드는 깊이 보정 기능을 끈다.
    /**
     * @brief 깊이 이미지 기록을 끝내고 앞서 사용하던 기록 대상과 화면 영역을 되돌린다.
     * @details 깊이 보정을 끈다. 호출 전 켜져 있던 보정 상태는 저장하지 않으므로 기존 렌더링에서 별도로 사용하지 않아야 한다.
     */
    void End();

    // 입력: shader가 깊이 이미지를 읽을 슬롯 번호.
    /**
     * @brief 깊이 이미지를 GPU의 지정 슬롯에 연결한다.
     * @param slot shader의 이미지 입력이 가리킬 슬롯 번호.
     * @details shader의 해당 이미지 입력값에도 같은 번호를 설정해야 한다.
     */
    void Bind(std::uint32_t slot) const;

private:
    std::uint32_t m_Framebuffer = 0;

    std::uint32_t m_DepthTexture = 0;

    // 한 변의 픽셀 수.
    int m_Size = 2048;

    // Begin 전에 사용한 GPU 출력 대상과 {x, y, 너비, 높이} [pixel].
    int m_PreviousFramebuffer = 0;

    int m_PreviousViewport[4]{};
};
