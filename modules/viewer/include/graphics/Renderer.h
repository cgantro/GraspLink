#pragma once

#include <memory>
#include <glm/glm.hpp>

namespace PoseLink {

struct Transform;
struct Renderable;

/*
    OpenGL Draw Call 수행만 담당
*/
class Renderer {
public:
    // 반드시 Window/OpenGL Context가 생성된 이후 호출해야 한다.
    Renderer();
    ~Renderer();

    void Init();
    void BeginFrame();

    /*
        RenderSystem이 선택한 하나의 렌더링 대상을 실제로 그림
    */
    void Draw(
        const Transform& transform,
        const Renderable& renderable,
        const glm::mat4& view,
        const glm::mat4& projection
    );
};

} // namespace PoseLink
