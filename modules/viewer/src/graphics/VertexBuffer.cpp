#include "VertexBuffer.h"

#include <glad/glad.h>

VertexBuffer::VertexBuffer(const void* data, uint32_t size)
{
    // CPU 메모리의 byte 범위를 GPU 사본으로 복사한다. 이후 draw는 이 사본을 읽는다.
    glGenBuffers(1, &m_RendererID);
    glBindBuffer(GL_ARRAY_BUFFER, m_RendererID);

    // GL_STATIC_DRAW는 초기 업로드 후 드물게 바꾸는 데이터라는 사용 패턴 힌트다.
    glBufferData(GL_ARRAY_BUFFER, size, data, GL_STATIC_DRAW);
}

VertexBuffer::~VertexBuffer()
{
    if (m_RendererID != 0)
    {
        glDeleteBuffers(1, &m_RendererID);
        m_RendererID = 0;
    }
}

void VertexBuffer::Bind() const
{
    glBindBuffer(GL_ARRAY_BUFFER, m_RendererID);
}

void VertexBuffer::UnBind() const
{
    // GL_ARRAY_BUFFER의 현재 선택만 비운다. GPU 사본과 VAO의 attribute 설명은 남는다.
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}
