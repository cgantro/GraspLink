#pragma once

#include <memory>
#include "Pose.h"

namespace PoseLink {

//헤더에서는 실제 정의가 필요하지 않으므로 전방 선언.
// 불필요하게 Shader.h, VertexBuffer.h 등을 포함하지 않아도 된다.
class Shader;
class Texture;
class Mesh;
class Camera;
/**
 * @brief 렌더러
 * Shader, VAO, VBO, EBO, Texture 구성
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
    void Render(const Pose& pose);
    void EndFrame();

private:
    std::unique_ptr<Mesh> m_Mesh;
    std::shared_ptr<Shader> m_Shader;

    // 초기화 검증과 추후 pose visualization texture에 사용하는 RGBA texture.
     std::unique_ptr<Texture> m_Texture;
     // Viewer가 사용할 3D Camera.
    std::unique_ptr<Camera> m_Camera;
};

} // namespace PoseLink
