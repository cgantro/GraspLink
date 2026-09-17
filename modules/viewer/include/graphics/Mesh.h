#pragma once

#include <cstdint>
#include <memory>

#include <glm/glm.hpp>


class VertexArray;
class VertexBuffer;
class IndexBuffer;

// 준비된 정점과 인덱스를 GPU 버퍼로 옮기는 단순 mesh다.
struct Vertex
{
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec2 texCoord;
};

class Mesh final
{
public:
    Mesh(const Vertex* vertices, std::uint32_t vertexCount,
         const std::uint32_t* indices, std::uint32_t indexCount);
    ~Mesh();

    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;

    void Bind() const;
    void UnBind() const;
    std::uint32_t GetIndexCount() const;

    static std::unique_ptr<Mesh> CreateCube();

private:
    std::unique_ptr<VertexArray> vertexArray_;
    std::unique_ptr<VertexBuffer> vertexBuffer_;
    std::unique_ptr<IndexBuffer> indexBuffer_;
};
