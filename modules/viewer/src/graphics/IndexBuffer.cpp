#include "IndexBuffer.h"

#include <glad/glad.h>
#include <stdexcept>


IndexBuffer::IndexBuffer(const uint32_t* indices, uint32_t cnt) : m_Count(cnt){
    if(indices == nullptr) throw std::runtime_error(
        "IndexBuffer indices must not be null"
    );

    if(cnt == 0) throw std::runtime_error(
        "IndexBuffer count  must be Greater than 0"
    );

    // CPU 배열과 별개로 GPU에 보관할 index buffer 객체를 만든다.
   glGenBuffers(1,&m_RendererID);

   // EBO 연결은 현재 VAO에 기록되므로 Mesh가 먼저 VAO를 Bind한 상태여야 한다.
   glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,m_RendererID);

   // OpenGL 크기는 byte 단위지만 count와 draw 인수는 index 원소 개수다.
   glBufferData(
        GL_ELEMENT_ARRAY_BUFFER,
        m_Count * sizeof(uint32_t),
        indices,
        GL_STATIC_DRAW
   );

    // VAO가 선택된 채 EBO를 0에 바인딩하면 VAO의 EBO 연결도 끊어진다.
}

IndexBuffer::~IndexBuffer(){
    // GPU 자원은 만든 context가 살아 있고 현재 스레드에 설정된 동안 해제해야 한다.
    if(m_RendererID != 0){
        glDeleteBuffers(1,&m_RendererID);
        m_RendererID = 0;
    }
}

void IndexBuffer::Bind() const {glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,m_RendererID);}

void IndexBuffer::UnBind() const{
    // GL_ELEMENT_ARRAY_BUFFER는 VAO 상태이므로 0 연결은 현재 VAO의 저장된 EBO도 지운다.
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);
}
