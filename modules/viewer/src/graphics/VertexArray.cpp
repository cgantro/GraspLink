#include "VertexArray.h"

#include <glad/glad.h>

VertexArray::VertexArray()
{
    /*
        VAO는 vertex byte 자체를 저장하지 않는다.
        어떤 VBO/EBO가 연결되어 있고 각 attribute를 어떤 stride/offset으로 읽을지를 기억하는 상태 객체다.
        실제 glVertexAttribPointer 설정은 Mesh 생성 과정에서 VAO가 바인딩된 상태로 수행된다.
    */
    glGenVertexArrays(1, &m_RendererID);
}

VertexArray::~VertexArray()
{
    if (m_RendererID != 0)
    {
        glDeleteVertexArrays(1, &m_RendererID);
        m_RendererID = 0;
    }
}

void VertexArray::Bind() const
{
    glBindVertexArray(m_RendererID);
}

void VertexArray::UnBind() const
{
    glBindVertexArray(0);
}
