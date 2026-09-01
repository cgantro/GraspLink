#include "VertexBuffer.h"

#include <glad/glad.h> // OpenGL 함수 포인터 연결
// GLES는 OpenGL 표준 규격(임베디드)
// GLM 수학 라이브러리

VertexBuffer::VertexBuffer(const void* data, uint32_t size){
    // 1. OpenGL Buffer 객체 생성
    // Buffer 식별용 ID 생성
    glGenBuffers(1,&m_RendererID); // 아직 메모리 공간이 있는 것은 아니다.
    
    // 2. GL_ARRAY_BUFFER로 연결
    // 이후 glBufferData() call은 현재 Bind된 Buffer를 대상으로 동작한다.

    glBindBuffer(GL_ARRAY_BUFFER,m_RendererID);

    // 3. CPU 정점 데이터를 GPU Buffer로 복사
    // GL_STATIC_DRAW -> 정점 데이터가 자주 변경되지 않을 것이라고 힌트

    glBufferData(GL_ARRAY_BUFFER,size,data,GL_STATIC_DRAW);
}

VertexBuffer::~VertexBuffer(){
    if(m_RendererID != 0){
        glDeleteBuffers(1,&m_RendererID);
        m_RendererID = 0;
    }
}

void VertexBuffer::Bind() const{ glBindBuffer(GL_ARRAY_BUFFER,m_RendererID);}
void VertexBuffer::UnBind() const{ glBindBuffer(GL_ARRAY_BUFFER,0);}
