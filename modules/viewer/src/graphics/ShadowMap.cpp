#include "ShadowMap.h"

#include <glad/glad.h>

#include <stdexcept>

/**
 * @brief 깊이 텍스처와 depth-only framebuffer를 구성한다.
 * @details 24비트 정규화 깊이를 저장할 정사각형 텍스처를 만들고 광원 투영 밖의
 * 표본은 깊이 1로 읽히게 한다. framebuffer가 완성되지 않으면 이미 생성한 GPU
 * 객체를 직접 해제한 뒤 실패를 알린다. 유효한 OpenGL context가 필요하다.
 * @param size 텍스처 한 변의 크기 [pixel].
 * @throws std::invalid_argument size가 양수가 아닌 경우.
 * @throws std::runtime_error depth attachment를 포함한 framebuffer가 완성되지 않은 경우.
 */
ShadowMap::ShadowMap(int size)
    : m_Size(size)
{
    if (size <= 0)
        throw std::invalid_argument("Shadow map size must be positive");
    // 이유: 그림자 판정에는 광원이 본 깊이만 필요하므로 색 저장소는 만들지 않음.
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

    // 24비트 깊이는 맵 해상도에 따라 정규화된 광원 깊이를 양자화한다.
    // GL_LINEAR는 sampler2D 표본마다 주변 깊이를 보간한 뒤 조명 셰이더가 그 값을 비교하게 한다.
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);

    // 경계: 광원 영역 밖은 가장 먼 깊이(1)로 처리해 가짜 그림자를 방지.
    // 광원 투영 범위 밖의 깊이 1은 가장 먼 표본이므로 맵 경계 밖 조회가 가짜 그림자를 만들지 않는다.
    constexpr float borderColor[]{1.0F, 1.0F, 1.0F, 1.0F};
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);

    glBindFramebuffer(GL_FRAMEBUFFER, m_Framebuffer);
    glFramebufferTexture2D(
        GL_FRAMEBUFFER,
        GL_DEPTH_ATTACHMENT,
        GL_TEXTURE_2D,
        m_DepthTexture,
        0);

    // 제한: 깊이 전용 framebuffer이므로 색 읽기·쓰기는 끔.
    // 색 attachment가 없는 depth-only FBO이므로 색 읽기와 쓰기를 모두 비활성화한다.
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    {
        // 생성자에서 예외를 던지면 소멸자가 호출되지 않으므로 이미 만든 GL 객체 두 개를 여기서 해제한다.
        // 이전 framebuffer를 저장하지 않았으므로 실패 경로에서는 기본 framebuffer 0을 바인딩한다.
        // 수명: 생성자 실패 때는 소멸자가 실행되지 않으므로 만든 GPU 객체를 직접 해제.
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glDeleteTextures(1, &m_DepthTexture);
        glDeleteFramebuffers(1, &m_Framebuffer);
        m_DepthTexture = 0;
        m_Framebuffer = 0;
        throw std::runtime_error("Shadow framebuffer is incomplete");
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

/** @brief 소유한 OpenGL 텍스처와 framebuffer를 해제한다. 유효한 OpenGL context가 필요하다. */
ShadowMap::~ShadowMap()
{
    // 생성 때와 마찬가지로 GL 객체 삭제 시점에도 유효한 OpenGL context가 필요하다.
    if (m_DepthTexture != 0) glDeleteTextures(1, &m_DepthTexture);
    if (m_Framebuffer != 0) glDeleteFramebuffers(1, &m_Framebuffer);
}

/**
 * @brief depth pass용 렌더 대상을 활성화하고 깊이 버퍼를 초기화한다.
 * @details End에서 framebuffer와 viewport를 돌려놓을 수 있도록 두 상태를 저장한다.
 * depth test 자체는 호출자가 활성화해 두어야 한다.
 */
void ShadowMap::Begin()
{
    // 복원: 그림자를 그리기 전 렌더 대상과 viewport를 저장.
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &m_PreviousFramebuffer);
    glGetIntegerv(GL_VIEWPORT, m_PreviousViewport);

    glBindFramebuffer(GL_FRAMEBUFFER, m_Framebuffer);
    glViewport(0, 0, m_Size, m_Size);
    glClear(GL_DEPTH_BUFFER_BIT);

    // 이유: 저장 깊이를 조금 밀어 정밀도 오차로 표면에 생기는 점무늬 그림자를 완화.
    // factor는 면의 깊이 기울기에 비례하는 항을 조정하고, units는 구현체의 깊이 해상도 단위 오프셋을 더한다.
    // 양수 오프셋은 광원에서 더 먼 쪽으로 깊이를 밀어 acne를 줄이며, 너무 크면 그림자가 물체에서 떨어져 보인다.
    // 여기서는 면 컬링을 바꾸지 않으므로 호출자가 래스터화하는 모든 fill 면에 오프셋이 적용된다.
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0F, 1.0F);

}

/**
 * @brief 저장해 둔 framebuffer와 viewport를 복원한다.
 * @details polygon offset은 이전 값을 복원하지 않고 비활성화한다. 다른 OpenGL 상태는
 * 보존하지 않으므로 이 함수는 pass 진입 전 전체 상태 스냅샷을 복원하지 않는다.
 */
void ShadowMap::End()
{
    // Begin은 이전 polygon offset 활성 여부와 factor/units를 저장하지 않으므로 End는 기능을 끄기만 한다.
    // framebuffer와 viewport만 복원하며 depth test나 scissor 같은 나머지 GL 상태는 그대로 둔다.
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
