#pragma once
#include "assets/GraphicsTypes.h"

#include <cstdint>
#include <memory>

#include <glm/glm.hpp>


class VertexArray;
class VertexBuffer;
class IndexBuffer;

// Mesh는 CPU 메모리에 있는 정점/인덱스 데이터를 GPU가 그릴 수 있는 형태로 준비한다.
//
// 하나의 정점(Vertex)은 위치, 법선, 텍스처 좌표를 함께 가진다.
// 여러 정점은 VBO(Vertex Buffer Object)에 연속해서 저장되고,
// 삼각형을 구성할 정점 번호는 EBO(Index Buffer Object)에 저장된다.
// VAO(Vertex Array Object)는 "이 VBO/EBO를 이 attribute 배치로 읽어라"라는
// 연결 정보를 기억한다. 따라서 그릴 때는 Mesh의 VAO만 바인딩하면 된다.
//
// Vertex attribute layout:
//   location 0: position (vec3)
//   location 1: normal   (vec3)
//   location 2: texCoord (vec2)
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
