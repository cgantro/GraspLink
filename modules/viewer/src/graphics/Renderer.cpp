#include "Renderer.h"

#include "Material.h"
#include "Mesh.h"
#include "Shader.h"
#include "components/RenderComponents.h"
#include "ShadowMap.h"
#include "MultisampleFramebuffer.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glad/glad.h>

#include <cstdint>
#include <iostream>
Renderer::Renderer() = default;
Renderer::~Renderer() = default;

void Renderer::Init(int framebufferWidth, int framebufferHeight)
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);

    m_MSAAFramebuffer =std::make_unique<MultisampleFramebuffer>(framebufferWidth,framebufferHeight,4);

    m_ShadowMap = std::make_unique<ShadowMap>(2048);
    m_ShadowShader = Shader::Create("shaders/ShadowDepth.glsl");

    m_LightDirection = glm::normalize(m_LightDirection);

    const glm::vec3 target{0.0F, 0.7F, 0.0F};
    const glm::vec3 lightPosition =
        target + m_LightDirection * 4.0F;

    const glm::mat4 lightView = glm::lookAt(
        lightPosition,
        target,
        glm::vec3{0.0F, 1.0F, 0.0F});

    const glm::mat4 lightProjection = glm::ortho(
        -2.5F, 2.5F,
        -2.5F, 2.5F,
        0.1F, 10.0F);

    m_LightSpaceMatrix = lightProjection * lightView;
}

void Renderer::BeginFrame()
{
    m_MSAAFramebuffer->Bind();

    glClearColor(0.1F, 0.1F, 0.1F, 1.0F);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    GLint sampleBuffers = 0;
    GLint samples = 0;

    glGetIntegerv(GL_SAMPLE_BUFFERS, &sampleBuffers);
    glGetIntegerv(GL_SAMPLES, &samples);

    std::cout
    << "[MSAA FBO] sample buffers: "
    << sampleBuffers
    << ", samples: "
    << samples
    << '\n';

}
void Renderer::EndFrame()
{
    m_MSAAFramebuffer->ResolveToDefault();
}

void Renderer::Resize(int width, int height)
{
    if (m_MSAAFramebuffer)
        m_MSAAFramebuffer->Resize(width, height);
}

void Renderer::Draw(const glm::mat4& model,
                    const MeshFilter& meshFilter,
                    const MeshRenderer& meshRenderer,
                    const glm::mat4& view,
                    const glm::mat4& projection,
                    const glm::vec3& cameraPosition)
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
    meshRenderer.shader->SetFloat3(
        "u_CameraPosition",cameraPosition);
    meshRenderer.shader->SetFloat3(
        "u_LightDirection",m_LightDirection);

    meshRenderer.shader->SetMat4(
        "u_LightSpaceMatrix",m_LightSpaceMatrix);

    constexpr int shadowTextureSlot = 7;

    m_ShadowMap->Bind(shadowTextureSlot);

    meshRenderer.shader->SetInt(
        "u_ShadowMap",shadowTextureSlot);

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

void Renderer::BeginShadowPass()
{
    m_ShadowMap->Begin();

    m_ShadowShader->Bind();

    m_ShadowShader->SetMat4(
        "u_LightSpaceMatrix",
        m_LightSpaceMatrix);
}

void Renderer::DrawShadow(
    const glm::mat4& model,
    const MeshFilter& meshFilter)
{
    if (!meshFilter.mesh)
    {
        return;
    }

    m_ShadowShader->SetMat4(
        "u_Model",
        model);

    meshFilter.mesh->Bind();

    const std::size_t indexCount =
        meshFilter.indexCount == 0U
            ? meshFilter.mesh->GetIndexCount()
            : meshFilter.indexCount;

    const std::size_t indexOffset =
        meshFilter.indexOffset *
        sizeof(std::uint32_t);

    glDrawElements(
        GL_TRIANGLES,
        static_cast<GLsizei>(
            indexCount),
        GL_UNSIGNED_INT,
        reinterpret_cast<const void*>(
            indexOffset));

    meshFilter.mesh->UnBind();
}

void Renderer::EndShadowPass()
{
    m_ShadowShader->UnBind();

    m_ShadowMap->End();
}