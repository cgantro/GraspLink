#include "Renderer.h"

#include "Material.h"
#include "Mesh.h"
#include "Shader.h"
#include "components/RenderComponents.h"

#include <glad/glad.h>

#include <cstdint>
#include <iostream>
Renderer::Renderer() = default;
Renderer::~Renderer() = default;

void Renderer::Init()
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);

    GLint sampleBuffers = 0;
    GLint samples = 0;

    glGetIntegerv(GL_SAMPLE_BUFFERS, &sampleBuffers);
    glGetIntegerv(GL_SAMPLES, &samples);

    std::cout
        << "[Renderer] Sample buffers: "
        << sampleBuffers
        << ", MSAA samples: "
        << samples
        << '\n';
}

void Renderer::BeginFrame()
{
    glClearColor(0.1F, 0.1F, 0.1F, 1.0F);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::Draw(const glm::mat4& model,
                    const MeshFilter& meshFilter,
                    const MeshRenderer& meshRenderer,
                    const glm::mat4& view,
                    const glm::mat4& projection)
{
    if (!meshFilter.mesh || !meshRenderer.shader || !meshRenderer.material)
    {
        return;
    }

    meshRenderer.shader->Bind();
    meshRenderer.shader->SetMat4("u_Model", model);
    meshRenderer.shader->SetMat4("u_View", view);
    meshRenderer.shader->SetMat4("u_Projection", projection);
    meshRenderer.shader->SetFloat4(
        "u_BaseColorFactor", meshRenderer.material->BaseColorFactor());
    meshRenderer.shader->SetFloat(
        "u_MetallicFactor", meshRenderer.material->MetallicFactor());
    meshRenderer.shader->SetFloat(
        "u_RoughnessFactor", meshRenderer.material->RoughnessFactor());

    meshFilter.mesh->Bind();
    const std::size_t indexCount = meshFilter.indexCount == 0U
        ? meshFilter.mesh->GetIndexCount()
        : meshFilter.indexCount;
    const std::size_t indexOffset =
        meshFilter.indexOffset * sizeof(std::uint32_t);
    glDrawElements(
        GL_TRIANGLES,
        static_cast<GLsizei>(indexCount),
        GL_UNSIGNED_INT,
        reinterpret_cast<const void*>(indexOffset));
    meshFilter.mesh->UnBind();
    meshRenderer.shader->UnBind();
}
