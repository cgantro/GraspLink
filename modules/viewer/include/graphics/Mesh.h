#pragma once

#include "assets/GraphicsTypes.h"

#include <cstdint>
#include <memory>

#include <glm/glm.hpp>

class VertexArray;
class VertexBuffer;
class IndexBuffer;

/**
 * @brief CPU Vertex/Index 데이터를 OpenGL VAO/VBO/EBO 조합으로 업로드하고 소유하는 GPU Mesh.
 *
 * @details
 * - VBO: interleaved Vertex byte 데이터 저장
 * - EBO: uint32 vertex index 저장
 * - VAO: VBO byte layout을 shader attribute location에 연결한 상태 저장
 *
 * 현재 attribute layout:
 * - location 0: position vec3
 * - location 1: normal vec3
 * - location 2: texCoord vec2
 *
 * Vertex::tangent는 CPU-side 데이터에는 존재하지만 아직 shader attribute로 연결하지 않는다.
 * Vertex position의 공간 단위는 source asset을 그대로 따른다. 현재 HCR controller-ready asset은 meter [m].
 *
 * @todo [FUTURE] normal mapping 구현 시 tangent attribute를 연결하고 vec4 handedness까지 보존한다.
 */
class Mesh final
{
public:
    /**
     * @brief CPU vertex/index 배열을 정적 GPU buffer로 업로드해 Mesh를 생성한다.
     * @param vertices Vertex 배열 시작 주소.
     * @param vertexCount Vertex 원소 개수. byte 수가 아니다.
     * @param indices uint32 index 배열 시작 주소.
     * @param indexCount index 원소 개수. byte 수가 아니다.
     */
    Mesh(
        const Vertex* vertices,
        std::uint32_t vertexCount,
        const std::uint32_t* indices,
        std::uint32_t indexCount);

    /** @brief VAO/VBO/EBO RAII 객체를 해제한다. */
    ~Mesh();

    /** @brief GPU object ownership 중복을 막기 위해 copy construction을 금지한다. */
    Mesh(const Mesh&) = delete;

    /** @brief GPU object ownership 중복을 막기 위해 copy assignment를 금지한다. */
    Mesh& operator=(const Mesh&) = delete;

    /** @brief 이 Mesh의 VAO를 바인딩해 vertex input/EBO 상태를 draw에 사용할 수 있게 한다. */
    void Bind() const;

    /** @brief 현재 VAO binding을 해제한다. */
    void UnBind() const;

    /** @return 전체 Mesh의 index 원소 개수. */
    std::uint32_t GetIndexCount() const;

    /**
     * @brief 그래픽스 디버깅용 단위 Cube Mesh를 생성한다.
     * @return GPU buffer를 소유하는 unique_ptr<Mesh>.
     * @note Cube의 실제 scene 크기는 Entity Scale/Model matrix로 조정할 수 있다.
     */
    static std::unique_ptr<Mesh> CreateCube();

private:
    /** @brief vertex attribute binding state owner. */
    std::unique_ptr<VertexArray> vertexArray_;

    /** @brief interleaved Vertex data buffer owner. */
    std::unique_ptr<VertexBuffer> vertexBuffer_;

    /** @brief uint32 index buffer owner. */
    std::unique_ptr<IndexBuffer> indexBuffer_;
};
