#pragma once

#include <cstdint>

/**
 * @brief 정점 재사용을 위한 OpenGL Index Buffer(EBO)를 RAII로 관리한다.
 *
 * @details
 * Vertex Buffer에 정점을 한 번 저장하고 Index Buffer에는 "몇 번째 정점을 사용할지"만 저장하면
 * 삼각형끼리 공유하는 정점을 중복 저장하지 않아도 된다. glDrawElements가 이 buffer를 사용한다.
 */
/*
 * [추가 그래픽스 용어 설명]
 * - Index: Vertex 배열에서 몇 번째 정점을 사용할지 가리키는 번호.
 * - EBO(Element Buffer Object): OpenGL에서 index 배열을 GPU에 저장하는 Buffer Object.
 * - Buffer Object: GPU 메모리에 올린 데이터 덩어리를 OpenGL ID로 가리키는 객체.
 * - Bind: "이후 OpenGL 명령이 이 객체를 대상으로 동작한다"고 현재 상태에 연결하는 것.
 * - RAII: C++ 객체 생성 시 GPU 자원을 만들고 소멸자에서 자동 해제해 수명 관리를 묶는 방식.
 *
 * cnt와 m_Count는 byte 수가 아니라 index 원소 개수다.
 * m_RendererID는 index 값이 아니라 OpenGL이 발급한 Buffer Object ID다.
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
    // OpenGL glGenBuffers가 발급한 GPU Buffer Object handle. 0은 아직 생성되지 않은 상태로 사용한다.
    uint32_t m_RendererID = 0;

    // GPU에 업로드한 index 원소 개수. byte 크기가 아니다.
    uint32_t m_Count = 0;
};
