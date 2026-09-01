#include "Mesh.h"

#include "VertexArray.h"
#include "VertexBuffer.h"
#include "IndexBuffer.h"

#include <glad/glad.h>
#include <stdexcept>

namespace PoseLink
{
Mesh::Mesh(const Vertex* vertices, uint32_t vertexCount,
        const uint32_t* indices, uint32_t indexCount){
    if (vertices == nullptr || vertexCount  == 0)
        throw std::runtime_error("Mesh has no vertices");

    if (indices == nullptr || indexCount == 0)
        throw std::runtime_error("Mesh has no indices");

    
    /*
        VAO 먼저 Bind
        이후 설정하는 Vertex Att와 EBO Binding을 VAO가 기억한다.
    */
   
    m_VertexArray = std::make_unique<VertexArray>();
    m_VertexArray->Bind();

   /*
        실제 Vertex 데이터를 GPU로 복사
        현재는 [x][y][z][u][v]
   */
    m_VertexBuffer = std::make_unique<VertexBuffer>(
        vertices,sizeof(Vertex)*vertexCount 
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
        Px Py Pz U V
        l0

        Pos는 float 3개 사용
    */

    glEnableVertexAttribArray(0); // loc 0
    glVertexAttribPointer(
        0,                  // Shader의 layout(location = 0)
        3,                  // x, y, z
        GL_FLOAT,
        GL_FALSE,
        sizeof(Vertex),  // 다음 Vertex까지의 거리
        // Vertex 시작 위치부터 읽음
         reinterpret_cast<void*>(
            offsetof(Vertex, position)
        )            
    );

     /*
        Vertex Attribute 1 = Texture Coordinate

        [Px][Py][U][V]
                 ↑
                 location 1

        float 2개(Px, Py)를 건너뛴 위치에서 시작한다.
    */

    // offsetof
    // Vertex 구조체 시작 주소에서 texCoord까지 몇 Byte 떨어져 있어
    
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(
        1,
        2,
        GL_FLOAT,
        GL_FALSE,
        sizeof(Vertex),
        reinterpret_cast<void*>(
            offsetof(Vertex, texCoord)
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

std::unique_ptr<Mesh> Mesh::CreateCube(){
    /*
        Cube는 꼭짓점만 보면 8개임
        그러나 각 Face마다 Vertex따로 만듦

        공간상 물리적 위치(Position)가 같더라도, 
        그 정점이 속한 면(Face)마다 맵핑되는 UV 좌표(텍스처 좌표)가 다를 수 있기 때문
    */
   const Vertex vertices[] = {

        // Front (+Z)
        {{-0.5f, -0.5f,  0.5f}, {0.0f, 0.0f}},
        {{ 0.5f, -0.5f,  0.5f}, {1.0f, 0.0f}},
        {{ 0.5f,  0.5f,  0.5f}, {1.0f, 1.0f}},
        {{-0.5f,  0.5f,  0.5f}, {0.0f, 1.0f}},

        // Back (-Z)
        {{ 0.5f, -0.5f, -0.5f}, {0.0f, 0.0f}},
        {{-0.5f, -0.5f, -0.5f}, {1.0f, 0.0f}},
        {{-0.5f,  0.5f, -0.5f}, {1.0f, 1.0f}},
        {{ 0.5f,  0.5f, -0.5f}, {0.0f, 1.0f}},

        // Left (-X)
        {{-0.5f, -0.5f, -0.5f}, {0.0f, 0.0f}},
        {{-0.5f, -0.5f,  0.5f}, {1.0f, 0.0f}},
        {{-0.5f,  0.5f,  0.5f}, {1.0f, 1.0f}},
        {{-0.5f,  0.5f, -0.5f}, {0.0f, 1.0f}},

        // Right (+X)
        {{ 0.5f, -0.5f,  0.5f}, {0.0f, 0.0f}},
        {{ 0.5f, -0.5f, -0.5f}, {1.0f, 0.0f}},
        {{ 0.5f,  0.5f, -0.5f}, {1.0f, 1.0f}},
        {{ 0.5f,  0.5f,  0.5f}, {0.0f, 1.0f}},

        // Top (+Y)
        {{-0.5f,  0.5f,  0.5f}, {0.0f, 0.0f}},
        {{ 0.5f,  0.5f,  0.5f}, {1.0f, 0.0f}},
        {{ 0.5f,  0.5f, -0.5f}, {1.0f, 1.0f}},
        {{-0.5f,  0.5f, -0.5f}, {0.0f, 1.0f}},

        // Bottom (-Y)
        {{-0.5f, -0.5f, -0.5f}, {0.0f, 0.0f}},
        {{ 0.5f, -0.5f, -0.5f}, {1.0f, 0.0f}},
        {{ 0.5f, -0.5f,  0.5f}, {1.0f, 1.0f}},
        {{-0.5f, -0.5f,  0.5f}, {0.0f, 1.0f}}
    };

    /*
        OpenGL은 Triangle을 기본 단위로 그린다.

        한 Face는 사각형이므로 Triangle 두 개가 필요하다.

            3 ------ 2
            |      / |
            |    /   |
            |  /     |
            |/       |
            0 ------ 1

        Triangle 1 = 0, 1, 2
        Triangle 2 = 2, 3, 0

        Cube 전체:
            6 Face
            × 2 Triangle
            × 3 Index
            = 36 Index
    */

    const uint32_t indices[] = {
         0,  1,  2,   2,  3,  0,   // Front
         4,  5,  6,   6,  7,  4,   // Back
         8,  9, 10,  10, 11,  8,   // Left
        12, 13, 14,  14, 15, 12,   // Right
        16, 17, 18,  18, 19, 16,   // Top
        20, 21, 22,  22, 23, 20    // Bottom
    };

    return std::make_unique<Mesh>(
        vertices,
        24,
        indices,
        36
    );
}
} // namespace PoseLink
