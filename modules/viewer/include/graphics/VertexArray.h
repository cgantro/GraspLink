#pragma once

#include <cstdint>

/**
 * @brief OpenGL VAO(Vertex Array Object)를 RAII로 관리한다.
 *
 * @details
 * VBO는 raw byte 배열이므로 GPU는 각 byte가 position/normal/UV 중 무엇인지 스스로 알 수 없다.
 * VAO는 vertex attribute location, component count/type, stride/offset과 연결된 buffer 상태를 기억한다.
 * Mesh를 그릴 때 VAO를 bind하면 해당 vertex input 구성이 복원된다.
 */
class VertexArray
{
public:
    /** @brief 새로운 OpenGL VAO object를 생성한다. */
    VertexArray();

    /** @brief 소유한 VAO object를 삭제한다. */
    ~VertexArray();

    /** @brief OpenGL object ID의 중복 ownership을 막기 위해 copy construction을 금지한다. */
    VertexArray(const VertexArray&) = delete;

    /** @brief OpenGL object ID의 중복 ownership을 막기 위해 copy assignment를 금지한다. */
    VertexArray& operator=(const VertexArray&) = delete;

    /** @brief 이 VAO를 현재 vertex array state로 바인딩한다. */
    void Bind() const;

    /** @brief 현재 VAO binding을 0으로 해제한다. */
    void UnBind() const;

private:
    /** @brief OpenGL vertex array object ID. 0은 생성 전/해제 후 상태. */
    uint32_t m_RendererID = 0;
};
