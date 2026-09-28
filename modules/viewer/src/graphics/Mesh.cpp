#include "Mesh.h"

#include "IndexBuffer.h"
#include "VertexArray.h"
#include "VertexBuffer.h"

#include <glad/glad.h>

#include <cstddef>
#include <limits>
#include <stdexcept>


Mesh::Mesh(const Vertex* vertices, std::uint32_t vertexCount,
           const std::uint32_t* indices, std::uint32_t indexCount)
{
    if (vertices == nullptr || vertexCount == 0U) { throw std::invalid_argument("Mesh has no vertices"); }
    if (indices == nullptr || indexCount == 0U) { throw std::invalid_argument("Mesh has no indices"); }

    // VAO를 먼저 바인딩한다.
    // OpenGL의 vertex attribute 설정은 현재 바인딩된 VAO에 기록되므로,
    // 이 순서를 지키지 않으면 다른 VAO에 잘못된 정점 배치가 저장될 수 있다.
    vertexArray_ = std::make_unique<VertexArray>();
    vertexArray_->Bind();

    // Vertex 배열 전체를 GPU의 VBO로 복사한다.
    // byteCount는 정점 개수가 아니라 정점 데이터 전체의 바이트 수다.
    const std::size_t byteCount = sizeof(Vertex) * static_cast<std::size_t>(vertexCount);
    if (byteCount > std::numeric_limits<std::uint32_t>::max())
    {
        throw std::runtime_error("Mesh vertex data exceeds the GPU buffer limit");
    }
    vertexBuffer_ = std::make_unique<VertexBuffer>(vertices, static_cast<std::uint32_t>(byteCount));
    vertexBuffer_->Bind();

    // 인덱스 배열을 GPU의 EBO로 복사한다.
    // EBO는 VAO에 연결되므로 VAO가 바인딩된 상태에서 생성/바인딩해야 한다.
    indexBuffer_ = std::make_unique<IndexBuffer>(indices, indexCount);

    // Vertex 구조체는 [position][normal][texCoord] 순서로 한 정점 안에
    // 여러 attribute를 나란히 저장하는 interleaved layout이다.
    //
    // stride는 현재 정점에서 다음 정점까지 이동하는 바이트 수다.
    // sizeof(Vertex)를 사용하면 padding을 포함한 실제 구조체 간격과 일치한다.
    // offset은 한 정점의 시작 주소에서 각 attribute가 시작하는 위치다.
    // offsetof(Vertex, field)는 C++ 구조체에서 그 위치를 바이트 단위로 계산한다.
    //
    // location 번호는 셰이더의 layout(location = N)과 반드시 일치해야 한다.
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, position)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, normal)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, texCoord)));

    // glVertexAttribPointer를 호출하는 순간 현재 VBO와 attribute 설정이 VAO에 기록된다.
    // 그래서 이후 VBO를 해제해도 VAO가 정점 데이터를 읽는 방법은 유지된다.
    // 반면 EBO 바인딩은 VAO 상태의 일부이므로 VAO가 바인딩된 동안 연결을 유지한다.
    vertexBuffer_->UnBind();
    vertexArray_->UnBind();
}

Mesh::~Mesh() = default;

void Mesh::Bind() const { vertexArray_->Bind(); }
void Mesh::UnBind() const { vertexArray_->UnBind(); }
std::uint32_t Mesh::GetIndexCount() const { return indexBuffer_->GetCount(); }

std::unique_ptr<Mesh> Mesh::CreateCube()
{
    const Vertex vertices[] = {
        {{-0.5F, -0.5F,  0.5F}, {0.0F, 0.0F, 1.0F}, {0.0F, 0.0F}},
        {{ 0.5F, -0.5F,  0.5F}, {0.0F, 0.0F, 1.0F}, {1.0F, 0.0F}},
        {{ 0.5F,  0.5F,  0.5F}, {0.0F, 0.0F, 1.0F}, {1.0F, 1.0F}},
        {{-0.5F,  0.5F,  0.5F}, {0.0F, 0.0F, 1.0F}, {0.0F, 1.0F}},
        {{ 0.5F, -0.5F, -0.5F}, {0.0F, 0.0F,-1.0F}, {0.0F, 0.0F}},
        {{-0.5F, -0.5F, -0.5F}, {0.0F, 0.0F,-1.0F}, {1.0F, 0.0F}},
        {{-0.5F,  0.5F, -0.5F}, {0.0F, 0.0F,-1.0F}, {1.0F, 1.0F}},
        {{ 0.5F,  0.5F, -0.5F}, {0.0F, 0.0F,-1.0F}, {0.0F, 1.0F}},
        {{-0.5F, -0.5F, -0.5F}, {-1.0F, 0.0F, 0.0F}, {0.0F, 0.0F}},
        {{-0.5F, -0.5F,  0.5F}, {-1.0F, 0.0F, 0.0F}, {1.0F, 0.0F}},
        {{-0.5F,  0.5F,  0.5F}, {-1.0F, 0.0F, 0.0F}, {1.0F, 1.0F}},
        {{-0.5F,  0.5F, -0.5F}, {-1.0F, 0.0F, 0.0F}, {0.0F, 1.0F}},
        {{ 0.5F, -0.5F,  0.5F}, {1.0F, 0.0F, 0.0F}, {0.0F, 0.0F}},
        {{ 0.5F, -0.5F, -0.5F}, {1.0F, 0.0F, 0.0F}, {1.0F, 0.0F}},
        {{ 0.5F,  0.5F, -0.5F}, {1.0F, 0.0F, 0.0F}, {1.0F, 1.0F}},
        {{ 0.5F,  0.5F,  0.5F}, {1.0F, 0.0F, 0.0F}, {0.0F, 1.0F}},
        {{-0.5F,  0.5F,  0.5F}, {0.0F, 1.0F, 0.0F}, {0.0F, 0.0F}},
        {{ 0.5F,  0.5F,  0.5F}, {0.0F, 1.0F, 0.0F}, {1.0F, 0.0F}},
        {{ 0.5F,  0.5F, -0.5F}, {0.0F, 1.0F, 0.0F}, {1.0F, 1.0F}},
        {{-0.5F,  0.5F, -0.5F}, {0.0F, 1.0F, 0.0F}, {0.0F, 1.0F}},
        {{-0.5F, -0.5F, -0.5F}, {0.0F,-1.0F, 0.0F}, {0.0F, 0.0F}},
        {{ 0.5F, -0.5F, -0.5F}, {0.0F,-1.0F, 0.0F}, {1.0F, 0.0F}},
        {{ 0.5F, -0.5F,  0.5F}, {0.0F,-1.0F, 0.0F}, {1.0F, 1.0F}},
        {{-0.5F, -0.5F,  0.5F}, {0.0F,-1.0F, 0.0F}, {0.0F, 1.0F}}
    };
    const std::uint32_t indices[] = {
        0,1,2, 2,3,0, 4,5,6, 6,7,4, 8,9,10, 10,11,8,
        12,13,14, 14,15,12, 16,17,18, 18,19,16, 20,21,22, 22,23,20
    };
    return std::make_unique<Mesh>(vertices, 24U, indices, 36U);
}
