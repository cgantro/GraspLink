#pragma once

#include <cstdint>
namespace PoseLink
{

/**
 * @brief indexBuffer
 * @details 3D 그래픽스에서 정점의 중복을 제거하기위해 index 번호만 모아둔 공간
 *          예시로, 사각형을 그릴때 정점 6개가 필요함
 *          인덱스 버퍼를 사용하면, 정점 4개만 저장하고, 인덱스 버퍼에는 그 정점들의 번호만 저장해 사각형 그리라 명령
 * 
 * @author 홍윤표
 */
class IndexBuffer{
public:
    /*
        indices : CPU 메모리 상의 index 배열의 시작 주소
        cnt : index 개수
    */
    IndexBuffer(const uint32_t* indices, uint32_t cnt);
    ~IndexBuffer();

    // 복사 방지
    IndexBuffer(const IndexBuffer&) = delete;
    IndexBuffer& operator=(const IndexBuffer&) = delete;

    void Bind() const;
    void UnBind() const;

    uint32_t GetCount() const{return m_Count;}
private:
    // glGenBuffers()를 통해 OpenGL이 발급한 Buffer Object ID.
    uint32_t m_RendererID = 0;
    // 저장하고 있는 Index 개수
    uint32_t m_Count = 0;
};  
} // namespace PoseLink
