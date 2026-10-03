#include "MultisampleFramebuffer.h"

#include <glad/glad.h>

#include <algorithm>
#include <stdexcept>

MultisampleFramebuffer::MultisampleFramebuffer(
    int width,
    int height,
    int samples)
    : m_Width(width),
      m_Height(height),
      m_Samples(samples)
{
    GLint maxSamples = 1;
    glGetIntegerv(GL_MAX_SAMPLES, &maxSamples);

    m_Samples = std::min(m_Samples, static_cast<int>(maxSamples));

    if (m_Samples < 2)
        throw std::runtime_error("OpenGL multisampling is not supported");

    Create();
}

MultisampleFramebuffer::~MultisampleFramebuffer()
{
    Destroy();
}

void MultisampleFramebuffer::Create()
{
    glGenFramebuffers(1, &m_Framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, m_Framebuffer);

    // Color
    glGenTextures(1, &m_ColorTexture);
    glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, m_ColorTexture);

    glTexImage2DMultisample(
        GL_TEXTURE_2D_MULTISAMPLE,
        m_Samples,
        GL_RGBA8,
        m_Width,
        m_Height,
        GL_TRUE);

    glFramebufferTexture2D(
        GL_FRAMEBUFFER,
        GL_COLOR_ATTACHMENT0,
        GL_TEXTURE_2D_MULTISAMPLE,
        m_ColorTexture,
        0);

    // Depth + Stencil
    glGenRenderbuffers(1, &m_DepthStencilBuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, m_DepthStencilBuffer);

    glRenderbufferStorageMultisample(
        GL_RENDERBUFFER,
        m_Samples,
        GL_DEPTH24_STENCIL8,
        m_Width,
        m_Height);

    glFramebufferRenderbuffer(
        GL_FRAMEBUFFER,
        GL_DEPTH_STENCIL_ATTACHMENT,
        GL_RENDERBUFFER,
        m_DepthStencilBuffer);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    {
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        throw std::runtime_error("MSAA framebuffer is incomplete");
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void MultisampleFramebuffer::Destroy()
{
    if (m_DepthStencilBuffer != 0)
        glDeleteRenderbuffers(1, &m_DepthStencilBuffer);

    if (m_ColorTexture != 0)
        glDeleteTextures(1, &m_ColorTexture);

    if (m_Framebuffer != 0)
        glDeleteFramebuffers(1, &m_Framebuffer);

    m_DepthStencilBuffer = 0;
    m_ColorTexture = 0;
    m_Framebuffer = 0;
}

void MultisampleFramebuffer::Bind() const
{
    glBindFramebuffer(GL_FRAMEBUFFER, m_Framebuffer);
    glViewport(0, 0, m_Width, m_Height);
}

void MultisampleFramebuffer::ResolveToDefault() const
{
    glBindFramebuffer(GL_READ_FRAMEBUFFER, m_Framebuffer);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

    glBlitFramebuffer(
        0, 0, m_Width, m_Height,
        0, 0, m_Width, m_Height,
        GL_COLOR_BUFFER_BIT,
        GL_NEAREST);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void MultisampleFramebuffer::Resize(int width, int height)
{
    if (width <= 0 || height <= 0)
        return;

    if (width == m_Width && height == m_Height)
        return;

    m_Width = width;
    m_Height = height;

    Destroy();
    Create();
}