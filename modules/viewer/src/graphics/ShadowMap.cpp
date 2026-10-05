#include "ShadowMap.h"

#include <glad/glad.h>

#include <stdexcept>

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

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);

    // 경계: 광원 영역 밖은 가장 먼 깊이(1)로 처리해 가짜 그림자를 방지.
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
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    {
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

ShadowMap::~ShadowMap()
{
    if (m_DepthTexture != 0) glDeleteTextures(1, &m_DepthTexture);
    if (m_Framebuffer != 0) glDeleteFramebuffers(1, &m_Framebuffer);
}

void ShadowMap::Begin()
{
    // 복원: 그림자를 그리기 전 렌더 대상과 viewport를 저장.
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &m_PreviousFramebuffer);
    glGetIntegerv(GL_VIEWPORT, m_PreviousViewport);

    glBindFramebuffer(GL_FRAMEBUFFER, m_Framebuffer);
    glViewport(0, 0, m_Size, m_Size);
    glClear(GL_DEPTH_BUFFER_BIT);

    // 이유: 저장 깊이를 조금 밀어 정밀도 오차로 표면에 생기는 점무늬 그림자를 완화.
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0F, 1.0F);

}

void ShadowMap::End()
{
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
