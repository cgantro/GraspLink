#pragma once

#include <memory>

//헤더에서는 실제 정의가 필요하지 않으므로 전방 선언.
// 불필요하게 Shader.h, VertexBuffer.h 등을 포함하지 않아도 된다.
class Shader;
class VertexBuffer;
class VertexArray;
class Texture;
/**
 * @brief 렌더러
 * Shader, VAO, VBO 구성
 * 현재 단계에서는 Fullscreen Quad만 렌더링
 * 
 * 이후 추가
 * - Texture
 * - Video Frame rendering
 * - Overlay Rendering
 * - FrameBuffer
 * - Post Processing
 */
class Renderer {
public:
    // 반드시 Window/OpenGL Context가 생성된 이후 호출해야 한다.
    Renderer();
    ~Renderer();

    void Init();
    void BeginFrame();

    // 현재 바인드된 pose/texture visualization을 화면에 렌더링한다.
    // PoseLink는 방송 Scene 계층을 사용하지 않는다.
    void Render();
    void EndFrame();

private:
    std::shared_ptr<Shader> m_Shader;
    std::unique_ptr<VertexBuffer> m_VertexBuffer;
    std::unique_ptr<VertexArray> m_VertexArray;

    // 초기화 검증과 추후 pose visualization texture에 사용하는 RGBA texture.
     std::unique_ptr<Texture> m_Texture;
};
