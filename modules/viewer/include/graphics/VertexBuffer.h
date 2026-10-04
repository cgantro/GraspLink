#pragma once

#include <cstdint>

/**
 * @brief 정점 byte 데이터를 GPU Vertex Buffer Object(VBO)에 업로드하고 소유한다.
 *
 * @details
 * CPU가 매 vertex를 매 frame 전달하는 대신 모델 생성 시 정점 배열을 GPU VRAM에 한 번 업로드한다.
 * 이후 Draw에서는 Buffer ID와 attribute layout만 사용하므로 CPU-GPU 전송량을 줄일 수 있다.
 * 현재 Mesh는 position/normal/UV/tangent가 한 Vertex 구조체에 섞여 있는 interleaved layout을 사용한다.
 *
 * @todo [FUTURE] 동적으로 변경되는 mesh가 필요해지면 usage(GL_STATIC_DRAW/GL_DYNAMIC_DRAW)와 Update API를 분리한다.
 */
/*
 * [추가 그래픽스 용어 설명]
 * - Vertex(정점): 3D Mesh를 구성하는 점 하나. 위치 외에 normal/UV/tangent 같은 속성도 함께 가질 수 있다.
 * - VBO(Vertex Buffer Object): Vertex 데이터를 GPU 메모리에 저장하는 OpenGL Buffer Object.
 * - VRAM: GPU가 그래픽 데이터를 저장하는 메모리.
 * - Interleaved layout: 한 Vertex의 position/normal/UV 등을 한 구조체에 이어 붙여 저장하는 방식.
 * - Attribute: Shader의 vertex input으로 전달되는 position, normal, texCoord 같은 정점 속성.
 *
 * 생성자의 size는 Vertex 개수가 아니라 전체 데이터 크기 [byte]다.
 * m_RendererID는 메모리 주소가 아니라 OpenGL이 발급한 Buffer Object ID다.
 */
class VertexBuffer
{
public:
    /**
     * @brief CPU byte 영역을 GL_ARRAY_BUFFER에 업로드한다.
     * @param data 업로드할 데이터 시작 주소.
     * @param size byte 단위 전체 크기.
     */
    VertexBuffer(const void* data, uint32_t size);

    /** @brief 소유한 OpenGL Buffer Object를 삭제한다. */
    ~VertexBuffer();

    VertexBuffer(const VertexBuffer&) = delete;
    VertexBuffer& operator=(const VertexBuffer&) = delete;

    /** @brief 이 VBO를 GL_ARRAY_BUFFER에 바인딩한다. */
    void Bind() const;

    /** @brief GL_ARRAY_BUFFER binding을 해제한다. */
    void UnBind() const;

private:
    // OpenGL이 발급한 VBO handle. 0은 아직 유효한 GPU buffer가 없다는 초기값으로 사용한다.
    uint32_t m_RendererID = 0;
};
