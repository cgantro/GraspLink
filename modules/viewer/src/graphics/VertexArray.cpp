#include "VertexArray.h"

#include <glad/glad.h>

VertexArray::VertexArray()
{
    // VAO handle만 만든다. attribute 형식과 EBO 연결은 Mesh가 이 VAO를 선택한 뒤 설정한다.
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
    // 현재 VAO 선택만 바꾼다. VAO 안에 기록한 attribute 설명과 EBO 연결은 보존한다.
    glBindVertexArray(0);
}
