#pragma once

#include <cstdint>
#include <memory>

namespace PoseLink
{
class VertexArray;
class VertexBuffer;
class IndexBuffer;

class Mesh{
public:
    /*
        vertices CPU상의 Vertex 시작주소
        vertexSize Vertex 데이터 크기(Byte)
        indices index 배열 시작주소
        indexCount index 개수

        Mesh는 전달받은 데이터를 GPU Buffer로 복사
        생성자가 끝난뒤 원본 배열이 사라져도 문제 X
    */
    Mesh(const float* vertices, uint32_t vertexSize,
        const uint32_t* indices, uint32_t indexCount);
    ~Mesh();

    // 복사 X
    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;
    

    /*
        이 Mesh의 VAO를 OpenGL Context에 bind
        VAO 내부 :
            - Vertex 특징 설정
            - VBO 정보
            - EBO 정보
    */
    void Bind() const;
    void UnBind() const;

    uint32_t GetIndexCount() const;
private:
    std::unique_ptr<VertexArray> m_VertexArray;
    std::unique_ptr<VertexBuffer> m_VertexBuffer;
    std::unique_ptr<IndexBuffer> m_IndexBuffer;
};
} // namespace poseLink
