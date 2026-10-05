#pragma once

#include <cstdint>

/**
 * @brief 정점 데이터를 읽는 형식과 index buffer 연결을 draw에 제공한다.
 * @details
 * VAO(Vertex Array Object)는 정점이나 index의 byte 데이터를 보관하지 않는다. Mesh 생성 중 설정한
 * attribute의 형식·stride·정점 안의 byte offset과 EBO 연결을 기억한다. 실제 데이터는 VertexBuffer와
 * IndexBuffer가 GPU 메모리에 보관한다.
 *
 * OpenGL은 VAO가 연결된 상태에서 attribute와 GL_ELEMENT_ARRAY_BUFFER 연결을 설정하면 그 상태를
 * 해당 VAO에 기록한다. VAO가 선택된 채 EBO를 0에 바인딩하면 저장된 연결도 끊긴다. draw할 때 이 VAO를
 * 선택하면 기억한 정점 형식과 EBO를 함께 사용할 수 있다.
 */
class VertexArray
{
public:
    /** @brief VAO를 만든다. 유효한 현재 OpenGL context가 필요하다. */
    VertexArray();

    /** @brief OpenGL context가 현재 유효한 동안 VAO를 해제한다. */
    ~VertexArray();

    VertexArray(const VertexArray&) = delete;
    VertexArray& operator=(const VertexArray&) = delete;

    /** @brief 이 VAO의 정점 형식과 EBO 연결을 현재 OpenGL 상태로 선택한다. */
    void Bind() const;

    /** @brief 현재 VAO 선택을 해제한다. 저장된 설정과 GPU 데이터는 지워지지 않는다. */
    void UnBind() const;

private:
    // OpenGL VAO 이름(ID).
    uint32_t m_RendererID = 0;
};
