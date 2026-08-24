#pragma once

#include <cstdint>

/**
 * @brief 정점 데이터를 GPU Buffer에 업로드 한다.
 * @author 홍윤표
 * 
 * 과거엔 매 프레임마다 CPU가 GPU에게 그리라(""여기 점 찍어, 저기 점 찍어")고 명령을 하나씩 명령을 보냈다.
 * CPU와 GPU 사이의 데이터 버스 병목 때문에 성능이 매우 낮았다.
 * 
 * VBO를 통해 3D 모델의 데이터를 초기화 할 때, 딱 한번만 GPU RAM(VrAM)으로 통째로 업로드 한다.
 * 그 후 렌더링 시에, GPU의 메모리 주소의 Vertex로 그려라라는 명령을 보냄 
 * 빠름
 * 
 * 내부에는 
 * Position, Color, Normal(빛 계산을 위한 법선 벡터)
 * Texture Coordinates(UV): 이미지를 입히기 위한 2D 텍스처 좌표(U,V) 저장
 * 
 * 두 가지 방식이 있다
 * 1. 구조체 배열 방식
 *      하나의 정점에 대한 모든 속성을 나란히 묶는다.
 *      GPU 캐시 효율(메모리 지역성) 극대화
 *      메모리 형태 [xyz][rgb][uv] [xyz][rgb][uv]
 * 2. 배열의 구조체 방식
 *      속성별로 메모리를 따로 모아두거나, 아예 VBO를 여러 개 만드는 방식
 *      특정 속성만 자주 업데이트해야할 때 제한적 사용
 *      메모리 형태 [xyz][xyz][xyz]
 */   
class VertexBuffer{
public:
    // 정점 데이터를 GPU Buff에 업로드 한다
    // data -> CPU 메모리의 정점 데이터 시작주소
    // size -> 전체 데이터 크기
    VertexBuffer(const void* data, uint32_t size);
    ~VertexBuffer();

    // 복사 금지
    // 같은 OpenGL Buffer를 여러 객체가 소유하면 안돼용
    VertexBuffer(const VertexBuffer&) = delete;
    VertexBuffer& operator=(const VertexBuffer&) = delete;

    // VBO를 GL_ARRAY_BUFFER에 연결/해제한다.
    void Bind() const;
    void UnBind() const;
private:
    // glGenBuffers()가 생성한 OpenGL Buffer ID
    uint32_t m_RendererID = 0;
};