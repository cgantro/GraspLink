#include "graphics/VertexArray.h"

#include <glad/glad.h>

VertexArray::VertexArray(){
    // VAO 생성
    glGenVertexArrays(1,&m_RendererID);
}

VertexArray::~VertexArray(){
    if(m_RendererID != 0){
        glDeleteVertexArrays(1,&m_RendererID);
        m_RendererID = 0;
    }
}

void VertexArray::Bind() const{glBindVertexArray(m_RendererID);}
void VertexArray::UnBind() const{glBindVertexArray(0);}
