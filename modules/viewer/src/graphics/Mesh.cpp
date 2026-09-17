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

    vertexArray_ = std::make_unique<VertexArray>();
    vertexArray_->Bind();

    const std::size_t byteCount = sizeof(Vertex) * static_cast<std::size_t>(vertexCount);
    if (byteCount > std::numeric_limits<std::uint32_t>::max())
    {
        throw std::runtime_error("Mesh vertex data exceeds the GPU buffer limit");
    }
    vertexBuffer_ = std::make_unique<VertexBuffer>(vertices, static_cast<std::uint32_t>(byteCount));
    vertexBuffer_->Bind();
    indexBuffer_ = std::make_unique<IndexBuffer>(indices, indexCount);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, position)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, normal)));
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void*>(offsetof(Vertex, texCoord)));

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
