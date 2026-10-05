#pragma once

#include <cstdint>

/**
 * @brief 정점의 연속된 byte 데이터를 GPU에 복사해 draw에서 재사용한다.
 * @details
 * 이 buffer는 CPU 배열을 소유하지 않고 생성 시 내용을 GL_ARRAY_BUFFER로 업로드한다. 업로드가 끝난 뒤
 * 원본 CPU 배열을 보관할 필요는 없다. UnBind는 현재 buffer 선택만 해제하며 GPU 사본은 유지한다.
 * GPU 자원은 소멸자에서 삭제된다.
 *
 * Mesh는 전체 Vertex 배열의 byte 수를 전달하고 GL_STATIC_DRAW 사용 힌트를 준다. 위치·법선·UV를 읽는
 * 형식은 VAO에 설정한다. stride는 Vertex 사이 byte 간격이고 offset은 한 Vertex 안 속성의 시작 위치다.
 */
class VertexBuffer
{
public:
    /**
     * @brief 정점 데이터를 GPU buffer에 복사한다.
     * @param data 복사할 원본 주소. 업로드 후 호출자가 원본을 보관할 필요는 없다.
     * @param size 복사할 전체 크기 [byte]. 정점 원소 개수가 아니다.
     * @details GL_STATIC_DRAW는 초기 업로드 뒤 드물게 변경하는 사용 패턴 힌트다. data와 size의 유효성은
     * 별도로 검사하지 않으므로 호출자가 올바른 범위를 전달해야 한다. 유효한 현재 OpenGL context가 필요하다.
     */
    VertexBuffer(const void* data, uint32_t size);

    /** @brief 현재 유효한 OpenGL context에서 GPU buffer를 해제한다. */
    ~VertexBuffer();

    VertexBuffer(const VertexBuffer&) = delete;
    VertexBuffer& operator=(const VertexBuffer&) = delete;

    /** @brief 이 buffer를 GL_ARRAY_BUFFER의 현재 대상으로 선택한다. */
    void Bind() const;

    /** @brief GL_ARRAY_BUFFER의 현재 선택을 해제한다. GPU 데이터는 유지된다. */
    void UnBind() const;

private:
    // OpenGL buffer 이름(ID).
    uint32_t m_RendererID = 0;
};
