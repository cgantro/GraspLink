#include "Renderer.h"

#include "Material.h"
#include "Mesh.h"
#include "components/RenderComponents.h"
#include "Shader.h"
#include "components/TransformComponents.h"

#include <glad/glad.h>

#include <cstdint>


Renderer::Renderer() = default;
Renderer::~Renderer() = default;

void Renderer::Init()
{
    glEnable(GL_DEPTH_TEST);
}

void Renderer::BeginFrame()
{
    glClearColor(0.1F, 0.1F, 0.1F, 1.0F);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::Draw(const Transform& transform, const Renderable& renderable,
                    const glm::mat4& view, const glm::mat4& projection)
{
    if (!renderable.mesh || !renderable.shader || !renderable.material) { return; }

    renderable.shader->Bind();
    renderable.shader->SetMat4("u_Model", transform.GetMatrix());
    renderable.shader->SetMat4("u_View", view);
    renderable.shader->SetMat4("u_Projection", projection);
    renderable.shader->SetFloat4("u_BaseColorFactor", renderable.material->BaseColorFactor());
    renderable.shader->SetFloat("u_MetallicFactor", renderable.material->MetallicFactor());
    renderable.shader->SetFloat("u_RoughnessFactor", renderable.material->RoughnessFactor());
    renderable.mesh->Bind();
    const std::size_t indexCount = renderable.indexCount == 0U
        ? renderable.mesh->GetIndexCount() : renderable.indexCount;
    const std::size_t indexOffset = renderable.indexOffset * sizeof(std::uint32_t);
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indexCount), GL_UNSIGNED_INT,
                   reinterpret_cast<const void*>(indexOffset));
    renderable.mesh->UnBind();
    renderable.shader->UnBind();
}
