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

    /*
        1. OpenGL Buffer Object 생성
        
        Index 데이터가 GPU에 올라가는 것은 아니다.
        GL이 관리할 Buffer Object의 ID만 발급
    */
   glGenBuffers(1,&m_RendererID);

   /*
        2. 이 Buffer를 Index Buffer로 바인드
   */

   // EBO / IndexBuffer의 바인드
   glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,m_RendererID);

   /*
        3. CPU의 indices들을 GPU Buffer로 복사
   */

   glBufferData(
        GL_ELEMENT_ARRAY_BUFFER,
        m_Count * sizeof(uint32_t),
        indices,
        GL_STATIC_DRAW
   );

    /*
    주의: 여기서 Index Buffer(GL_ELEMENT_ARRAY_BUFFER)를 바로 Unbind하면 안 됩니다.
    
    OpenGL에서 Index Buffer의 바인딩 상태는 '현재 바인딩된 VAO 내부'에 저장됩니다.
    만약 VAO가 바인딩된 상태에서 glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)을 호출해 버리면,
    방금 연결한 Index Buffer가 VAO에서 해제(Disconnect)되어 버립니다.
    
    즉, "이 VAO는 Index Buffer를 사용하지 않는다"고 기록되므로 일부러 Unbind하지 않고 남겨둡니다.
    
    VBO(GL_ARRAY_BUFFER)는 glVertexAttribPointer가 호출되는 순간 그 설정이 VAO에 복사되므로, 호출 직후 바로 Unbind 해도 안전하다.

    */
}

IndexBuffer::~IndexBuffer(){
    /*
        glGenBuffers()로 생성한 GPU 버퍼는 C++ 객체가 사라진다고 바로 사라지지 않는다.
        IndexBuffer 객체가 GPU Buffer의 소유자 -> 소멸자에서 삭제
    */

    if(m_RendererID != 0){
        glDeleteBuffers(1,&m_RendererID);
        m_RendererID = 0;
    }
}

/*
    glDrawElements가 사용할 Index Buffer 지정
    GL -> 상태 머신 -> Buffer ID를 Draw 함수에 직접 지정 x

    먼저 Bind 하고 그 뒤 Draw 호출
*/
void IndexBuffer::Bind() const {glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,m_RendererID);}

/*
    Binding을 0으로 변경

    주의:

    이 Binding은 VAO 상태의 일부 -> VAO가 Bind된 상태에서 호출하지 않는 것이 좋다

    Mesh를 구성할 때는 일반적으로

        VAO BIND
        VBO 설정
        EBO Bind
        VAO UnBind 순이다.
*/
void IndexBuffer::UnBind() const{glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,0);}
