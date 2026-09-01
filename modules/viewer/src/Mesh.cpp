#include "Mesh.h"

#include "VertexArray.h"
#include "VertexBuffer.h"
#include "IndexBuffer.h"

#include <glad/glad.h>
#include <stdexcept>

namespace PoseLink
{
Mesh::Mesh(const float* vertices, uint32_t vertexSize,
        const uint32_t* indices, uint32_t indexCount){
    if (vertices == nullptr)
        throw std::runtime_error(
            "Mesh vertices must not be null"
        );

    if (vertexSize == 0)
        throw std::runtime_error(
            "Mesh vertex size must be greater than 0"
        );

    if (indices == nullptr)
        throw std::runtime_error(
            "Mesh indices must not be null"
        );

    if (indexCount == 0)
        throw std::runtime_error(
            "Mesh index count must be greater than 0"
        );

    
    /*
        VAO 먼저 Bind
        이후 설정하는 Vertex Att와 EBO Binding을 VAO가 기억한다.
    */
   
    m_VertexArray = std::make_unique<VertexArray>();
    m_VertexArray->Bind();

   /*
        실제 Vertex 데이터를 GPU로 복사
        현재는 [x][y][u][v] float 4개
   */
   
    m_VertexBuffer = std::make_unique<VertexBuffer>(
        vertices,vertexSize
    );
    m_VertexBuffer->Bind();

    /*
        VAO의 Indexbuffer 생성
    */
    m_IndexBuffer = std::make_unique<IndexBuffer>(
        indices, indexCount
    );

    /*
        Vertex Att 0 = Position
        Px Py U V
        l0

        Pos는 float 2개 사용
    */

    glEnableVertexAttribArray(0); // loc 0
    glVertexAttribPointer(
        0,                  // Shader의 layout(location = 0)
        2,                  // x, y
        GL_FLOAT,
        GL_FALSE,
        4 * sizeof(float),  // 다음 Vertex까지의 거리
        nullptr             // Vertex 시작 위치부터 읽음
    );

     /*
        Vertex Attribute 1 = Texture Coordinate

        [Px][Py][U][V]
                 ↑
                 location 1

        float 2개(Px, Py)를 건너뛴 위치에서 시작한다.
    */
   glEnableVertexAttribArray(1);
   glVertexAttribPointer(
        1,
        2,                  // u, v
        GL_FLOAT,
        GL_FALSE,
        4 * sizeof(float),
        reinterpret_cast<void*>(
            2 * sizeof(float)
        )
    );

    m_VertexArray->UnBind();

    m_VertexBuffer->UnBind();
}
Mesh::~Mesh() = default;
void Mesh::Bind() const{
    /*
        VAO를 다시 Bind하면 생성 시 저장한
        - Vertex Att 설정
        - VBO, EBO 연결 정보 복원 가능
    */
    m_VertexArray->Bind();
}
void Mesh::UnBind() const{
    m_VertexArray->UnBind();
}

uint32_t Mesh::GetIndexCount() const{ return m_IndexBuffer->GetCount();}
} // namespace PoseLink
