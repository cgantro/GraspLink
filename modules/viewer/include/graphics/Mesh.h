#pragma once

#include "assets/GraphicsTypes.h"

#include <cstdint>
#include <memory>

#include <glm/glm.hpp>

class VertexArray;
class VertexBuffer;
class IndexBuffer;

// 역할: CPU 정점·index 배열을 GPU에 복사하고 정점 입력 설정과 함께 소유.
// 수명: Mesh를 공유하는 마지막 참조가 해제될 때까지 GL context가 살아 있어야 함.
// 제한: tangent는 저장하지만 Shader 입력에 연결하지 않아 normal mapping은 미지원.
class Mesh final
{
public:
    // 입력: vertexCount·indexCount는 원소 개수. CPU 배열은 생성 중 복사하며 포인터를 보관하지 않음.
    Mesh(
        const Vertex* vertices,
        std::uint32_t vertexCount,
        const std::uint32_t* indices,
        std::uint32_t indexCount);

    ~Mesh();

    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;

    // 반영: 이 Mesh의 정점 입력 설정과 index buffer를 렌더링에 연결.
    void Bind() const;

    void UnBind() const;

    std::uint32_t GetIndexCount() const;

    // 출력: Local 원점 중심, 지정 변 길이 [m]의 정육면체.
    static std::unique_ptr<Mesh> CreateCube(float sideLengthMeters = 1.0F);

private:
    // VAO: 정점 속성을 읽는 방법과 buffer 연결을 저장.
    std::unique_ptr<VertexArray> vertexArray_;

    // VBO: GPU 정점 데이터.
    std::unique_ptr<VertexBuffer> vertexBuffer_;

    // EBO: 삼각형이 참조할 정점 번호.
    std::unique_ptr<IndexBuffer> indexBuffer_;
};
