#pragma once

#include <cstdint>

/**
 * @brief OpenGL VAO(Vertex Array Object)를 RAII로 관리한다.
 *
 * @details
 * VBO는 byte 배열일 뿐이므로 GPU는 각 byte가 position인지 normal인지 알 수 없다.
 * VAO는 `location 0 = vec3 position`처럼 vertex attribute 해석 규칙과 연결된 buffer 상태를 기억한다.
 * Mesh를 그릴 때 VAO를 바인딩하면 해당 vertex input 구성이 복원된다.
 */
class VertexArray
{
public:
    /** @brief 새로운 OpenGL VAO를 생성한다. */
    VertexArray();

    /** @brief 소유한 VAO를 삭제한다. */
    ~VertexArray();

    VertexArray(const VertexArray&) = delete;
    VertexArray& operator=(const VertexArray&) = delete;

    /** @brief 이 VAO를 현재 vertex array state로 바인딩한다. */
    void Bind() const;

    /** @brief VAO binding을 해제한다. */
    void UnBind() const;

private:
    uint32_t m_RendererID = 0;
};
