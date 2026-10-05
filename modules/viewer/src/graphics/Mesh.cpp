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

    // VAO를 먼저 연결해야 뒤에서 지정하는 attribute 형식과 EBO 연결이 이 Mesh의 상태로 기록된다.
    vertexArray_ = std::make_unique<VertexArray>();
    vertexArray_->Bind();

    // buffer 업로드 API는 byte 크기를 받으므로 Vertex 원소 개수에 구조체 크기를 곱한다.
    const std::size_t byteCount = sizeof(Vertex) * static_cast<std::size_t>(vertexCount);
    if (byteCount > std::numeric_limits<std::uint32_t>::max())
    {
        throw std::runtime_error("Mesh vertex data exceeds the GPU buffer limit");
    }
    vertexBuffer_ = std::make_unique<VertexBuffer>(vertices, static_cast<std::uint32_t>(byteCount));
    vertexBuffer_->Bind();

    // EBO 연결은 현재 VAO 상태에 포함된다. VAO가 연결된 동안 생성·업로드해 draw 때 다시 쓸 수 있게 한다.
    indexBuffer_ = std::make_unique<IndexBuffer>(indices, indexCount);

    // Shader의 location 0/1/2에 각각 위치(vec3), 법선(vec3), UV(vec2)를 연결한다.
    // stride는 정점 사이 간격, offsetof는 한 Vertex 안에서 속성이 시작하는 byte 위치다.
    // sizeof/offsetof를 사용해 구조체 padding도 반영한다. tangent는 Vertex에 저장되지만 여기서 설정하지 않는다.
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, position)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, normal)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, texCoord)));

    // VBO의 전역 바인딩은 풀어도 VAO의 attribute 설정은 남는다. EBO 연결은 VAO 상태이므로 유지한다.
    vertexBuffer_->UnBind();
    vertexArray_->UnBind();
}

Mesh::~Mesh() = default;

// draw 호출 시 EBO에 저장된 uint32 index가 VAO의 VBO 정점들을 참조한다.
void Mesh::Bind() const { vertexArray_->Bind(); }
void Mesh::UnBind() const { vertexArray_->UnBind(); }
std::uint32_t Mesh::GetIndexCount() const { return indexBuffer_->GetCount(); }

std::unique_ptr<Mesh> Mesh::CreateCube(float sideLengthMeters)
{
    if (!std::isfinite(sideLengthMeters) || sideLengthMeters <= 0.0F)
        throw std::invalid_argument("Cube side length must be finite and > 0");

    // 면마다 네 정점을 둬 인접한 면이 서로 다른 평면 법선과 UV 사각형을 가질 수 있게 한다.
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
    // 각 면을 두 삼각형으로 나눈다. index는 byte 위치가 아니라 vertices 배열의 원소 번호다.
    const std::uint32_t indices[] = {
        0,1,2, 2,3,0, 4,5,6, 6,7,4, 8,9,10, 10,11,8,
        12,13,14, 14,15,12, 16,17,18, 18,19,16, 20,21,22, 22,23,20
    };
    // 좌표를 정육면체 중심에서 각 축 방향으로 sideLengthMeters / 2까지 늘린다.
    for (Vertex& vertex : vertices)
        vertex.position *= sideLengthMeters;

    return std::make_unique<Mesh>(vertices, 24U, indices, 36U);
}
