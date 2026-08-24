#pragma once

#include <memory>

//헤더에서는 실제 정의가 필요하지 않으므로 전방 선언.
// 불필요하게 Shader.h, VertexBuffer.h 등을 포함하지 않아도 된다.
class Shader;
class VertexBuffer;
class VertexArray;
class Scene;
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

    // Scene을 화면에 렌더링한다.
    // 지금은 실제 사용은 안함
    // 이후 Scene의 imageElement, TextElement, LowerThird 등을 순회한다.
    void Render(const Scene& scene);
    void EndFrame();

private:
    std::shared_ptr<Shader> m_Shader;
    std::unique_ptr<VertexBuffer> m_VertexBuffer;
    std::unique_ptr<VertexArray> m_VertexArray;
};
