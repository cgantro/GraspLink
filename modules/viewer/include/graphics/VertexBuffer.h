#pragma once

#include <cstdint>

/**
 * @brief 정점 byte 데이터를 GPU Vertex Buffer Object(VBO)에 업로드하고 소유한다.
 *
 * @details
 * CPU 정점 배열을 모델 생성 시 GPU VRAM에 업로드하고 이후 draw에서는 buffer ID/VAO layout을 재사용한다.
 * 현재 Mesh는 position/normal/UV/tangent가 하나의 Vertex 구조체에 섞여 있는 interleaved layout을 사용한다.
 *
 * @todo [FUTURE] 동적으로 변경되는 mesh가 필요해지면 usage(GL_STATIC_DRAW/GL_DYNAMIC_DRAW)와 Update API를 분리한다.
 */
class VertexBuffer
{
public:
    /**
     * @brief CPU byte 영역을 GL_ARRAY_BUFFER에 업로드한다.
     * @param data 업로드할 데이터 시작 주소.
     * @param size 전체 데이터 크기 [byte]. Vertex 개수와 혼동하지 않는다.
     */
    VertexBuffer(const void* data, uint32_t size);

    /** @brief 소유한 OpenGL Buffer Object를 삭제한다. */
    ~VertexBuffer();

    /** @brief OpenGL object ID의 중복 ownership을 막기 위해 copy construction을 금지한다. */
    VertexBuffer(const VertexBuffer&) = delete;

    /** @brief OpenGL object ID의 중복 ownership을 막기 위해 copy assignment를 금지한다. */
    VertexBuffer& operator=(const VertexBuffer&) = delete;

    /** @brief 이 VBO를 GL_ARRAY_BUFFER target에 바인딩한다. */
    void Bind() const;

    /** @brief GL_ARRAY_BUFFER binding을 0으로 해제한다. */
    void UnBind() const;

private:
    /** @brief OpenGL buffer object ID. 0은 생성 전/해제 후 상태. */
    uint32_t m_RendererID = 0;
};
