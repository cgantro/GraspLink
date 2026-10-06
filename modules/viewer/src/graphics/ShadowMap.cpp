#include "ShadowMap.h"

#include <glad/glad.h>

#include <stdexcept>

/**
 * @brief 그림자 계산에 사용할 깊이 Texture와 이를 기록하는 framebuffer를 만든다.
 * @details 광원 기준 투영 방향에서 앞뒤 순서를 나타내는 깊이를 24 bit로 저장하는 정사각형 Texture를 만든다. 이 값은 광원까지의 실제 직선거리가 아니다.
 * 광원 시야 바깥은 가장 먼 투영 깊이인 1로 처리한다. Framebuffer는 GPU가 색이나 깊이를 기록하는 대상으로 화면 창과 다르다.
 * 생성에 실패하면 이미 만든 GPU 객체를 직접 해제하고 오류를 알린다. 생성에는 GPU 명령을 실행할 OpenGL context가 현재 스레드에서 활성화되어야 한다.
 * @param size 깊이 Texture 한 변의 픽셀 수.
 * @throws std::invalid_argument size가 양수가 아닌 경우.
 * @throws std::runtime_error depth attachment를 포함한 framebuffer가 완성되지 않은 경우.
 */
ShadowMap::ShadowMap(int size)
    : m_Size(size)
{
    if (size <= 0)
        throw std::invalid_argument("Shadow map size must be positive");
    // 그림자를 판정할 때는 광원 기준 투영 깊이만 필요하므로 색상을 저장할 공간은 만들지 않는다.
    glGenFramebuffers(1, &m_Framebuffer);
    glGenTextures(1, &m_DepthTexture);

    glBindTexture(GL_TEXTURE_2D, m_DepthTexture);
    glTexImage2D(
        GL_TEXTURE_2D,
        0,
        GL_DEPTH_COMPONENT24,
        m_Size,
        m_Size,
        0,
        GL_DEPTH_COMPONENT,
        GL_FLOAT,
        nullptr);

    // 24-bit 저장은 투영 깊이를 제한된 단계로 기록하므로 아주 가까운 표면의 깊이 차이는 같은 숫자로 반올림될 수 있다.
    // GL_LINEAR는 이웃 깊이 표본을 섞어 조명 Shader가 경계 부근을 비교할 때 급격한 변화를 줄인다.
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);

    // 광원 투영 범위를 벗어난 위치는 가장 먼 깊이 1로 처리한다.
    // 그래서 Texture 경계 밖을 조회해도 실제 표면이 있는 것처럼 잘못된 그림자가 생기지 않는다.
    constexpr float borderColor[]{1.0F, 1.0F, 1.0F, 1.0F};
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);

    glBindFramebuffer(GL_FRAMEBUFFER, m_Framebuffer);
    glFramebufferTexture2D(
        GL_FRAMEBUFFER,
        GL_DEPTH_ATTACHMENT,
        GL_TEXTURE_2D,
        m_DepthTexture,
        0);

    // 이 framebuffer에는 깊이 Texture만 붙어 있다. 색상 저장소가 없으므로 색상 읽기와 쓰기를 모두 끈다.
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    {
        // 생성자에서 예외가 발생하면 소멸자가 호출되지 않는다. 따라서 이미 만든 Texture와 framebuffer를 여기서 직접 해제한다.
        // 이전 framebuffer를 보관하지 않았으므로 실패 경로에서는 기본 framebuffer ID 0을 선택한다.
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glDeleteTextures(1, &m_DepthTexture);
        glDeleteFramebuffers(1, &m_Framebuffer);
        m_DepthTexture = 0;
        m_Framebuffer = 0;
        throw std::runtime_error("Shadow framebuffer is incomplete");
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

/** @brief 소유한 OpenGL 깊이 Texture와 framebuffer를 해제한다. 삭제 시점에도 유효한 OpenGL context가 필요하다. */
ShadowMap::~ShadowMap()
{
    // 생성 때와 마찬가지로 GPU 객체를 삭제할 때도 이 객체가 만든 OpenGL context가 유효해야 한다.
    if (m_DepthTexture != 0) glDeleteTextures(1, &m_DepthTexture);
    if (m_Framebuffer != 0) glDeleteFramebuffers(1, &m_Framebuffer);
}

/**
 * @brief 광원에서 본 깊이를 기록할 대상을 선택하고 이전 깊이값을 지운다.
 * @details Framebuffer는 GPU가 그려진 깊이를 저장하는 대상이고 viewport는 그릴 창 픽셀의 사각 영역이다. End()에서 이전 값을 복원할 수 있도록 둘을 저장한다.
 * 깊이 비교는 앞뒤 표면을 판정하는 GPU 기능이며 이 함수가 켜지 않으므로 호출자가 미리 활성화해야 한다.
 */
void ShadowMap::Begin()
{
    // 그림자 깊이를 기록하기 전에 사용 중이던 framebuffer와 viewport를 기억한다.
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &m_PreviousFramebuffer);
    glGetIntegerv(GL_VIEWPORT, m_PreviousViewport);

    glBindFramebuffer(GL_FRAMEBUFFER, m_Framebuffer);
    glViewport(0, 0, m_Size, m_Size);
    glClear(GL_DEPTH_BUFFER_BIT);

    // 깊이값을 광원에서 더 먼 쪽으로 조금 밀어 반올림 차이가 표면에 점무늬 그림자로 나타나는 자기 가림을 줄인다.
    // factor는 삼각형 면의 기울기에 따른 보정량이고 units는 깊이 저장 정밀도에 맞춘 일정 오프셋이다.
    // 양수 오프셋이 너무 크면 그림자가 물체 표면에서 떨어져 보인다.
    // 여기서는 면 제거 설정을 바꾸지 않으므로 래스터화되는 모든 채워진 면에 오프셋이 적용된다.
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0F, 1.0F);

}

/**
 * @brief Begin() 전에 사용하던 framebuffer와 viewport를 다시 선택한다.
 * @details 다각형 깊이 오프셋은 삼각형 깊이에 작은 보정값을 더하는 기능이다. 이전 값을 저장하지 않았으므로 이 기능을 끈다.
 * Framebuffer와 viewport 두 상태 외에는 복원하지 않으므로 진입 전 OpenGL 전체 상태를 되돌리는 함수는 아니다.
 */
void ShadowMap::End()
{
    // Begin()은 다각형 깊이 오프셋의 이전 활성 여부와 보정값을 저장하지 않으므로 이 기능만 끈다.
    // framebuffer와 viewport만 복원한다. 깊이 검사와 화면 자르기 같은 나머지 OpenGL 상태는 그대로 둔다.
    glDisable(GL_POLYGON_OFFSET_FILL);
    glBindFramebuffer(GL_FRAMEBUFFER, static_cast<GLuint>(m_PreviousFramebuffer));
    glViewport(
        m_PreviousViewport[0],
        m_PreviousViewport[1],
        m_PreviousViewport[2],
        m_PreviousViewport[3]);
}

void ShadowMap::Bind(std::uint32_t slot) const
{
    glActiveTexture(GL_TEXTURE0 + slot);
    glBindTexture(GL_TEXTURE_2D, m_DepthTexture);
}
