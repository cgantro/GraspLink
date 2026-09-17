#pragma once
/**
 * @brief VAO
 * @author 홍윤표
 * VBO만 있으면 왜 안그려질까.
 * VBO는 정점 데이터만 들어있다 -> 이게 3D좌표인지, 색상인지, 구별할 수 없다.
 * VAO는 통역 역할
 */

#include <cstdint>



class VertexArray{
public:
    VertexArray();
    ~VertexArray();

    VertexArray(const VertexArray&) = delete;
    VertexArray& operator=(const VertexArray&) = delete;

    void Bind() const;
    void UnBind() const;
private:
    uint32_t m_RendererID = 0;
};
