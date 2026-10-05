#pragma once

#include <cstdint>

/**
 * @brief 정점 번호를 GPU에 복사해 glDrawElements가 삼각형 연결에 사용하게 한다.
 * @details
 * 각 uint32 index는 정점 buffer의 한 Vertex를 가리키는 원소 번호다. 업로드 byte 수는 원소 개수에
 * sizeof(uint32_t)를 곱한다. 이 객체는 GPU 사본을 소유하며 CPU 배열은 소유하지 않는다. UnBind는 현재
 * EBO 연결을 바꾸지만 GPU 사본을 해제하지 않는다.
 *
 * GL_ELEMENT_ARRAY_BUFFER 연결은 현재 VAO 상태에 기록된다. VAO가 선택된 채 EBO를 0에 바인딩하면 그
 * VAO의 연결도 사라진다. draw 때 같은 VAO를 선택하면 attribute 설명과 저장된 EBO를 함께 사용한다.
 */
class IndexBuffer
{
public:
    /**
     * @brief uint32 index 배열을 GPU에 복사한다.
     * @param indices 복사할 정점 번호 배열 주소.
     * @param cnt 배열의 index 원소 개수. byte 크기가 아니다.
     * @throws std::runtime_error indices가 null이거나 cnt가 0인 경우.
     * @details GL_STATIC_DRAW는 초기 업로드 뒤 드물게 변경하는 사용 패턴 힌트다. 개별 index의 범위는
     * 검사하지 않으므로 호출자가 유효한 정점 번호를 제공해야 한다. 유효한 현재 OpenGL context가 필요하다.
     */
    IndexBuffer(const uint32_t* indices, uint32_t cnt);

    /** @brief 현재 유효한 OpenGL context에서 GPU buffer를 해제한다. */
    ~IndexBuffer();

    IndexBuffer(const IndexBuffer&) = delete;
    IndexBuffer& operator=(const IndexBuffer&) = delete;

    /** @brief 이 buffer를 현재 VAO의 element array buffer로 연결한다. */
    void Bind() const;

    /** @brief 현재 VAO의 element array buffer 연결을 해제한다. VAO가 선택돼 있으면 저장된 상태도 바뀐다. */
    void UnBind() const;

    /** @brief 업로드한 index 원소 개수를 반환한다. 단위: uint32 index 개수. */
    uint32_t GetCount() const { return m_Count; }

private:
    // OpenGL buffer 이름(ID).
    uint32_t m_RendererID = 0;

    // draw에 쓰는 원소 개수. 업로드 byte 크기는 m_Count * sizeof(uint32_t).
    uint32_t m_Count = 0;
};
