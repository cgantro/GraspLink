#pragma once

#include <cstdint>

class Texture
{
public:

    // 2D RGBA Texture
    // 생성 시점에는 GPU 메모리 공간만 확보한다
    Texture(int width, int height);
    ~Texture();

    Texture(const Texture&) = delete;
    Texture& operator=(const Texture&) = delete;

    // CPU의 픽셀 데이터를 GPU Texture에 갱신
    void Update(const void* data);

    // 특정 Texture Unit에 연결
    // Bind(0) -> GL_TEXTURE0에 연결
    void Bind(uint32_t slot = 0) const;

    void UnBind() const;

    int GetWidth() const{return m_Width;}
    int GetHeight() const { return m_Height;}
private:
    // 실제 Texture 객체 생성 후, GPU 메모리 확보
    void CreateInternal();
private:
    uint32_t m_RendererID = 0;
    int m_Width = 0;
    int m_Height = 0;
};