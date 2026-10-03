#include "ShadowMap.h"

#include <glad/glad.h>

#include <stdexcept>

ShadowMap::ShadowMap(int size)
    : m_Size(size)
{
    /*
        Shadow map에는 색상 정보가 필요 없다.
        Light 관점에서 가장 가까운 표면의 depth만 저장하면 main pass에서 가려짐을 판정할 수 있다.
        따라서 depth texture 하나만 연결한 depth-only framebuffer를 만든다.
    */
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

    // Light frustum 밖을 depth=1(가장 멀리)로 취급해 경계 밖이 불필요하게 그림자가 되는 것을 피한다.
    constexpr float borderColor[]{1.0F, 1.0F, 1.0F, 1.0F};
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);

    glBindFramebuffer(GL_FRAMEBUFFER, m_Framebuffer);
    glFramebufferTexture2D(
        GL_FRAMEBUFFER,
        GL_DEPTH_ATTACHMENT,
        GL_TEXTURE_2D,
        m_DepthTexture,
        0);

    // depth-only FBO이므로 color read/draw target을 사용하지 않는다.
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
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
    // Shadow pass가 끝난 뒤 main pass 상태를 복원하기 위해 현재 FBO/viewport를 저장한다.
    glGetIntegerv(GL_FRAMEBUFFER_BINDING, &m_PreviousFramebuffer);
    glGetIntegerv(GL_VIEWPORT, m_PreviousViewport);

    glBindFramebuffer(GL_FRAMEBUFFER, m_Framebuffer);
    glViewport(0, 0, m_Size, m_Size);
    glClear(GL_DEPTH_BUFFER_BIT);

    /*
        같은 표면의 저장 depth와 비교 depth가 floating-point 정밀도 때문에 서로 싸우면 shadow acne가 생긴다.
        polygon offset으로 shadow pass depth를 조금 밀어 self-shadow artifact를 완화한다.
    */
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0F, 1.0F);

    // TODO(FUTURE): slope/constant bias는 scene scale과 light 설정에 맞춰 graphics settings로 분리한다.
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
