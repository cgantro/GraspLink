#include "Renderer.h"

#include "Camera.h"
#include "Mesh.h"
#include "Renderable.h"
#include "Shader.h"
#include "Texture.h"
#include "Transform.h"

#include <glad/glad.h>
#include <glm/glm.hpp>

#include <memory>

namespace PoseLink {

Renderer::Renderer() = default;
Renderer::~Renderer() = default;

void Renderer::Init() {
    /*
        Camera는 현재 단일 Viewer 고정 상태다.
        Scene의 object component로 취급하지 않고 기존 Renderer 소유 구조를 유지한다.
    */
    m_Camera = std::make_unique<Camera>(
        glm::vec3(2.0f, 2.0f, 3.0f),
        glm::vec3(0.0f, 0.0f, 0.0f),
        1280.0f / 720.0f);

    glEnable(GL_DEPTH_TEST);
}

void Renderer::BeginFrame() {
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::Render(flecs::world& world) {
    /*
        Camera는 모든 Entity가 공유하므로 frame마다 한 번만 계산한다.
        각 Entity마다 달라지는 값은 Transform::GetMatrix()가 만드는 model matrix뿐이다.
    */
    const glm::mat4 view = m_Camera->GetViewMatrix();
    const glm::mat4 projection = m_Camera->GetProjectionMatrix();

    /*
        flecs는 Transform과 Renderable을 동시에 가진 entity만 이 callback에 전달한다.

        따라서 Renderer는 CubeA/CubeB 같은 이름, entity ID, 생성 순서에 의존하지 않는다.
        새 Object도 두 component를 설정하면 이 query에 포함되고, component 또는 entity가 제거되면
        별도 Renderer 상태 정리 없이 다음 frame부터 제외된다.
    */
    /*
        flecs 4.x의 world.each<T>()는 component 하나를 받는 편의 함수다.
        두 component를 함께 조회할 때는 query를 먼저 만들고 query.each()로 순회한다.
    */
    const auto renderables = world.query<Transform, Renderable>();
    renderables.each(
        [&view, &projection](flecs::entity, Transform& transform, Renderable& renderable) {
            /*
                기존 단일 Cube 렌더링과 같은 OpenGL 순서다.
                차이는 Renderer 멤버가 아니라 현재 Entity의 Renderable에서 resource를 가져온다는 점이다.
            */
            renderable.shader->Bind();
            renderable.shader->SetMat4("u_Model", transform.GetMatrix());
            renderable.shader->SetMat4("u_View", view);
            renderable.shader->SetMat4("u_Projection", projection);

            renderable.texture->Bind(0);
            renderable.mesh->Bind();
            glDrawElements(
                GL_TRIANGLES,
                renderable.mesh->GetIndexCount(),
                GL_UNSIGNED_INT,
                nullptr);
            renderable.mesh->UnBind();
            renderable.texture->UnBind();
            renderable.shader->UnBind();
        });
}

void Renderer::EndFrame() {
}

} // namespace PoseLink
