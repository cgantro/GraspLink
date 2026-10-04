#pragma once

#include <cstdint>

/**
 * @brief 정점 재사용을 위한 OpenGL Index Buffer(EBO)를 RAII로 관리한다.
 *
 * @details
 * Vertex Buffer에 정점을 한 번 저장하고 Index Buffer에는 "몇 번째 정점을 사용할지"만 저장한다.
 * 현재 Mesh pipeline은 모든 index를 `uint32_t`로 정규화하며 glDrawElements의 index type도 이에 맞춘다.
 * index 값은 byte offset이 아니라 vertex element 번호다.
 */
class IndexBuffer
{
public:
    /**
     * @brief CPU uint32 index 배열을 GPU GL_ELEMENT_ARRAY_BUFFER로 업로드한다.
     * @param indices index 배열 시작 주소. 각 값은 Vertex 배열 원소 번호.
     * @param cnt 업로드할 index 원소 개수. byte 크기가 아니다.
     */
    IndexBuffer(const uint32_t* indices, uint32_t cnt);

    /** @brief 소유한 OpenGL Buffer Object를 해제한다. */
    ~IndexBuffer();

    /** @brief 동일 OpenGL object ID의 이중 삭제를 막기 위해 copy construction을 금지한다. */
    IndexBuffer(const IndexBuffer&) = delete;

    /** @brief 동일 OpenGL object ID의 이중 삭제를 막기 위해 copy assignment를 금지한다. */
    IndexBuffer& operator=(const IndexBuffer&) = delete;

    /** @brief 이 EBO를 현재 VAO의 GL_ELEMENT_ARRAY_BUFFER binding으로 설정한다. */
    void Bind() const;

    /** @brief GL_ELEMENT_ARRAY_BUFFER binding을 0으로 해제한다. */
    void UnBind() const;

    /** @return GPU에 업로드된 index 원소 개수. byte 수가 아니다. */
    uint32_t GetCount() const { return m_Count; }

private:
    /** @brief OpenGL이 발급한 buffer object ID. 0은 아직 생성되지 않았거나 해제된 상태. */
    uint32_t m_RendererID = 0;

    /** @brief 업로드된 uint32 index 원소 개수. */
    uint32_t m_Count = 0;
};
