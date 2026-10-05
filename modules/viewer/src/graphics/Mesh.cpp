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

    // 순서: 정점 입력 설정이 다른 Mesh에 기록되지 않도록 이 VAO를 먼저 연결.
    vertexArray_ = std::make_unique<VertexArray>();
    vertexArray_->Bind();

    // 단위: 정점 원소 개수를 GPU 복사에 필요한 byte 수로 변환.
    const std::size_t byteCount = sizeof(Vertex) * static_cast<std::size_t>(vertexCount);
    if (byteCount > std::numeric_limits<std::uint32_t>::max())
    {
        throw std::runtime_error("Mesh vertex data exceeds the GPU buffer limit");
    }
    vertexBuffer_ = std::make_unique<VertexBuffer>(vertices, static_cast<std::uint32_t>(byteCount));
    vertexBuffer_->Bind();

    // 순서: index buffer 연결은 VAO에 기록되므로 같은 VAO가 연결된 상태에서 복사.
    indexBuffer_ = std::make_unique<IndexBuffer>(indices, indexCount);

    // 배치: Shader location 0=위치, 1=법선, 2=UV. 간격·시작 offset은 byte 단위.
    // 이유: sizeof와 offsetof로 구조체 padding까지 반영해 GPU가 올바른 속성을 읽게 함.
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, position)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, normal)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, texCoord)));

    // 연결: VBO 바인딩을 풀어도 VAO의 정점 설정은 유지. EBO는 연결한 채 VAO를 해제.
    vertexBuffer_->UnBind();
    vertexArray_->UnBind();
}

Mesh::~Mesh() = default;

void Mesh::Bind() const { vertexArray_->Bind(); }
void Mesh::UnBind() const { vertexArray_->UnBind(); }
std::uint32_t Mesh::GetIndexCount() const { return indexBuffer_->GetCount(); }

std::unique_ptr<Mesh> Mesh::CreateCube(float sideLengthMeters)
{
    if (!std::isfinite(sideLengthMeters) || sideLengthMeters <= 0.0F)
        throw std::invalid_argument("Cube side length must be finite and > 0");

    Vertex vertices[] = {
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
    for (Vertex& vertex : vertices)
        vertex.position *= sideLengthMeters;

    return std::make_unique<Mesh>(vertices, 24U, indices, 36U);
}
