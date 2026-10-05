#pragma once

#include <cstdint>

/**
 * @brief 광원 시점에서 본 장면의 깊이를 2D 텍스처에 저장한다.
 * @details
 * 광원에서 가장 가까운 표면의 깊이를 저장해 그림자 판정에 사용한다. 깊이 texture와
 * framebuffer를 소유하고 생성·해제하며, 소멸까지 OpenGL context가 살아 있어야 한다.
 * Shadow pass는 이 클래스가 만든 depth-only framebuffer에 광원 카메라 기준으로
 * GL_DEPTH_COMPONENT24 depth attachment에 물체를 그려 가장 가까운 표면의 정규화
 * 깊이를 기록한다. 이후 일반 렌더링 셰이더는 광원 공간 좌표를 텍스처 좌표와 깊이로
 * 바꾸고 저장된 깊이와 비교해 가림을 판단한다. 현재 조명 셰이더는 bias를 뺀 깊이를
 * 5×5 이웃 표본과 비교해 그림자 표본 비율을 평균하는 수동 PCF를 사용한다. 비교 깊이에
 * bias를 적용해 같은 표면의 정밀도 오차로 생기는 shadow acne를 줄인다. bias가 지나치면
 * 그림자가 물체와 떨어져 보이는 Peter Panning 현상이 생길 수 있다.
 * 이 클래스는 깊이 맵의 해상도와 GPU 객체 수명, pass 중 framebuffer와 viewport 전환을
 * 관리한다. OpenGL 객체 생성·삭제와 메서드 호출에는 유효한 OpenGL context가 필요하다.
 * size는 정사각형 맵 한 변의 픽셀 수이며 기본값 2048은 2048×2048 depth texture를 뜻한다.
 * 텍스처 경계 바깥은 가장 먼 깊이인 1로 취급해 맵 영역 밖이 그림자로 판정되지 않게 한다.
 * @todo [FUTURE] 해상도와 polygon offset 값을 graphics settings에서 조정할 수 있게 한다.
 */
class ShadowMap final
{
public:
    // 입력: 정사각형 한 변의 크기 [pixel], 양수만 허용.
    /**
     * @brief 정사각형 깊이 텍스처와 depth-only framebuffer를 만든다.
     * @param size 한 변의 해상도 [pixel]. 양수여야 한다.
     * @throws std::invalid_argument size가 0 이하인 경우.
     * @throws std::runtime_error framebuffer가 완성되지 않은 경우.
     * @details GPU 객체를 생성하므로 유효한 OpenGL context에서 호출해야 한다.
     */
    explicit ShadowMap(int size = 2048);

    /** @brief 깊이 텍스처와 framebuffer를 해제한다. 유효한 OpenGL context가 필요하다. */
    ~ShadowMap();

    ShadowMap(const ShadowMap&) = delete;
    ShadowMap& operator=(const ShadowMap&) = delete;

    // 반영: 그림자용 framebuffer와 viewport를 설정하고 깊이를 초기화.
    /**
     * @brief 깊이 맵 렌더링을 시작한다.
     * @details 현재 framebuffer와 viewport를 저장한 뒤 이 객체의 framebuffer 및
     * 정사각형 viewport를 바인딩하고 깊이 버퍼를 지운다. 폴리곤 오프셋을 켜서
     * 깊이 비교 오차로 생기는 자기 그림자 흔적을 줄인다. 깊이 기록을 위해 호출
     * 시점에 OpenGL depth test가 활성화되어 있어야 한다. 호출자는 이 사이에 광원
     * 공간 변환을 적용해 그림자 대상 지오메트리를 그려야 한다.
     */
    void Begin();

    // 복원: Begin 전 framebuffer와 viewport. 표면 깊이 보정은 끔.
    /**
     * @brief 깊이 맵 렌더링을 끝내고 저장한 framebuffer와 viewport를 복원한다.
     * @details 폴리곤 오프셋을 비활성화한다. 이전 폴리곤 오프셋 상태는 저장하거나
     * 복원하지 않으므로 호출 전 다른 렌더링에서 이를 사용 중이지 않아야 한다.
     */
    void End();

    // 입력: Shader가 깊이 texture를 읽을 texture unit 번호.
    /**
     * @brief 깊이 텍스처를 지정한 texture unit에 바인딩한다.
     * @param slot 셰이더 sampler가 참조할 texture unit 번호.
     * @details 호출자는 sampler uniform도 같은 unit 번호로 설정해야 한다.
     */
    void Bind(std::uint32_t slot) const;

private:
    std::uint32_t m_Framebuffer = 0;

    std::uint32_t m_DepthTexture = 0;

    // 단위: 정사각형 한 변의 크기 [pixel].
    int m_Size = 2048;

    // 복원: Begin 전 렌더 대상과 {x, y, width, height} [pixel].
    int m_PreviousFramebuffer = 0;

    int m_PreviousViewport[4]{};
};
