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
/*
 * [추가 그래픽스 용어 설명]
 * - VAO(Vertex Array Object): 어떤 VBO를 어떤 vertex attribute 형식으로 읽을지 기억하는 OpenGL 상태 객체.
 * - Attribute Location: Shader의 vertex input 번호. 예: location 0=position, 1=normal.
 * - Vertex input layout: Vertex 구조체의 byte 배치를 Shader input과 연결하는 규칙.
 * - Bind: 이후 draw가 이 VAO의 vertex input 설정을 사용하도록 현재 OpenGL 상태에 선택하는 것.
 *
 * VAO 자체가 실제 Vertex 데이터를 저장하는 것은 아니다.
 * 데이터는 VBO/EBO에 있고 VAO는 "어떻게 읽을지"에 대한 연결 상태를 기억한다.
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
    // OpenGL이 발급한 VAO handle. 0은 OpenGL의 default/no-user-VAO 상태를 의미한다.
    uint32_t m_RendererID = 0;
};
