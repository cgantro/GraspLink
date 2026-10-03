#pragma once

#include <cstdint>

/**
 * @brief 정점 재사용을 위한 OpenGL Index Buffer(EBO)를 RAII로 관리한다.
 *
 * @details
 * Vertex Buffer에 정점을 한 번 저장하고 Index Buffer에는 "몇 번째 정점을 사용할지"만 저장하면
 * 삼각형끼리 공유하는 정점을 중복 저장하지 않아도 된다. glDrawElements가 이 buffer를 사용한다.
 */
class IndexBuffer
{
public:
    /**
     * @brief CPU index 배열을 GPU Element Array Buffer로 업로드한다.
     * @param indices uint32 index 배열 시작 주소.
     * @param cnt index 개수.
     */
    IndexBuffer(const uint32_t* indices, uint32_t cnt);

    /** @brief 소유한 OpenGL Buffer Object를 해제한다. */
    ~IndexBuffer();

    IndexBuffer(const IndexBuffer&) = delete;
    IndexBuffer& operator=(const IndexBuffer&) = delete;

    /** @brief 이 EBO를 현재 VAO의 GL_ELEMENT_ARRAY_BUFFER로 바인딩한다. */
    void Bind() const;

    /** @brief GL_ELEMENT_ARRAY_BUFFER binding을 해제한다. */
    void UnBind() const;

    /** @brief 업로드된 index 개수를 반환한다. */
    uint32_t GetCount() const { return m_Count; }

private:
    uint32_t m_RendererID = 0;
    uint32_t m_Count = 0;
};
