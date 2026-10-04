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
 * 현재 GPU에 연결된 attribute layout:
 * - location 0: position vec3
 * - location 1: normal vec3
 * - location 2: texCoord vec2
 *
 * Vertex에는 tangent vec3도 들어 있지만 현재 Mesh.cpp에서 shader attribute로 연결하지 않는다.
 * 즉 tangent는 향후 normal mapping을 위한 CPU-side 준비 데이터 상태다.
 *
 * @todo [FUTURE] normal mapping 구현 시 tangent를 shader attribute에 연결한다.
 * @todo [FUTURE] glTF tangent handedness까지 보존하기 위해 Vertex::tangent를 vec4로 확장한다.
 */
/*
 * [추가 그래픽스 용어 설명]
 * - Mesh: 3D 형상을 이루는 Vertex와 Triangle 연결 정보의 묶음.
 * - Vertex: 3D 점 하나와 position/normal/UV 같은 속성.
 * - Index: Triangle이 어떤 Vertex를 사용할지 가리키는 번호.
 * - Triangle: 대부분의 실시간 3D 렌더링에서 표면을 구성하는 세 정점짜리 기본 도형.
 * - VAO: VBO의 byte 구조를 Shader attribute에 어떻게 연결할지 기억하는 상태 객체.
 * - VBO: Vertex 데이터를 GPU에 저장하는 buffer.
 * - EBO: Index 데이터를 GPU에 저장하는 buffer.
 * - Normal: 표면이 어느 방향을 향하는지 나타내는 단위 방향벡터. 조명 계산에 사용한다.
 * - UV: 3D 표면의 점을 2D Texture의 위치와 연결하는 좌표.
 * - Tangent: 표면 위에서 UV의 U 방향과 연결되는 접선 벡터. Normal Mapping에서 사용한다.
 *
 * vertexCount/indexCount는 byte 수가 아니라 원소 개수다.
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
    // Vertex attribute layout과 VBO/EBO binding 상태를 기억한다.
    std::unique_ptr<VertexArray> vertexArray_;

    // 실제 Vertex byte 데이터를 GPU에 보관한다.
    std::unique_ptr<VertexBuffer> vertexBuffer_;

    // Triangle이 사용할 Vertex 번호(index)를 GPU에 보관한다.
    std::unique_ptr<IndexBuffer> indexBuffer_;
};
