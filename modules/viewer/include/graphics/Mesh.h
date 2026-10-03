#pragma once

#include "assets/GraphicsTypes.h"

#include <cstdint>
#include <memory>

#include <glm/glm.hpp>

class VertexArray;
class VertexBuffer;
class IndexBuffer;

/**
 * @brief CPU Vertex/Index 데이터를 OpenGL VAO/VBO/EBO 조합으로 소유하는 GPU Mesh.
 *
 * @details
 * VBO는 실제 정점 데이터를 보관하고, EBO는 삼각형이 참조할 정점 번호를 보관한다.
 * VAO는 "VBO의 각 byte를 position/normal/UV 중 무엇으로 해석할지"라는 attribute 상태를 기억한다.
 * 따라서 Draw 시 Mesh::Bind() 한 번으로 필요한 vertex input 상태를 복원할 수 있다.
 *
 * 현재 attribute layout:
 * - location 0: position vec3
 * - location 1: normal vec3
 * - location 2: texCoord vec2
 * - location 3: tangent vec3
 *
 * @todo [FUTURE] normal mapping을 정확히 지원할 때 glTF tangent handedness를 위해 tangent를 vec4로 확장한다.
 */
class Mesh final
{
public:
    /** @brief CPU vertex/index 배열을 GPU buffer로 업로드해 Mesh를 생성한다. */
    Mesh(
        const Vertex* vertices,
        std::uint32_t vertexCount,
        const std::uint32_t* indices,
        std::uint32_t indexCount);

    /** @brief VAO/VBO/EBO RAII 객체를 해제한다. */
    ~Mesh();

    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;

    /** @brief 이 Mesh의 VAO를 바인딩한다. */
    void Bind() const;

    /** @brief 현재 VAO binding을 해제한다. */
    void UnBind() const;

    /** @brief 전체 Mesh index 개수를 반환한다. */
    std::uint32_t GetIndexCount() const;

    /** @brief 그래픽스 디버깅용 단위 Cube Mesh를 생성한다. */
    static std::unique_ptr<Mesh> CreateCube();

private:
    std::unique_ptr<VertexArray> vertexArray_;
    std::unique_ptr<VertexBuffer> vertexBuffer_;
    std::unique_ptr<IndexBuffer> indexBuffer_;
};
