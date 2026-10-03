#include "VertexBuffer.h"

#include <glad/glad.h>

VertexBuffer::VertexBuffer(const void* data, uint32_t size)
{
    /*
        VBO(Vertex Buffer Object)는 CPU 메모리의 정점 byte 배열을 GPU가 반복해서 읽을 수 있는
        Buffer Object로 올려 두는 방식이다. 매 frame 정점 하나씩 CPU -> GPU로 보내는 대신
        초기화 시 한 번 업로드하고 Draw에서는 Buffer를 참조한다.

        OpenGL은 상태 머신이므로 glBufferData()에 Buffer ID를 직접 넘기지 않는다.
        먼저 glBindBuffer()로 "현재 GL_ARRAY_BUFFER"를 선택한 뒤 그 대상에 데이터를 업로드한다.
    */
    glGenBuffers(1, &m_RendererID);
    glBindBuffer(GL_ARRAY_BUFFER, m_RendererID);

    /*
        GL_STATIC_DRAW는 "자주 읽지만 내용 변경은 드물다"는 사용 패턴 힌트다.
        드라이버가 메모리 배치 전략을 선택할 때 참고할 수 있다.

        TODO(FUTURE): 동적 deformable mesh가 들어오면 usage와 부분 갱신 API를 별도로 둔다.
    */
    glBufferData(GL_ARRAY_BUFFER, size, data, GL_STATIC_DRAW);
}

VertexBuffer::~VertexBuffer()
{
    // OpenGL resource는 C++ 객체 수명과 자동 연결되지 않으므로 RAII destructor에서 명시적으로 삭제한다.
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
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}
