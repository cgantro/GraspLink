#include "Renderer.h"

#include "Mesh.h"
#include "Renderable.h"
#include "Shader.h"
#include "Texture.h"
#include "Transform.h"
#include <iostream>
#include <glad/glad.h>

namespace PoseLink {

Renderer::Renderer() = default;
Renderer::~Renderer() = default;

void Renderer::Init() {
    glEnable(GL_DEPTH_TEST);
}

void Renderer::BeginFrame() {
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::Draw(
    const Transform& transform,
    const Renderable& renderable,
    const glm::mat4& view,
    const glm::mat4& projection){
    /*
        잘못 구성된 Renderable은 Draw x
        현재는 Mesh, Shader, Texture 모두 필요
    */
    if(!renderable.mesh || !renderable.shader || !renderable.texture) return;

    renderable.shader->Bind();
    renderable.shader->SetMat4("u_Model", transform.GetMatrix());
    renderable.shader->SetMat4("u_View", view);

    /*
     현재 Camera에서 만들어진 공통 View/Projection
    */
    renderable.shader->SetMat4("u_Projection",projection);
    renderable.texture->Bind(0);
    renderable.mesh->Bind();

    glDrawElements(
        GL_TRIANGLES,
        renderable.mesh->GetIndexCount(),
        GL_UNSIGNED_INT,
        nullptr
    );

    renderable.mesh->UnBind();
    renderable.texture->UnBind();
    renderable.shader->UnBind();
}
} // namespace PoseLink
