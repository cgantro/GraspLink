#include "Renderer.h"

#include "Texture.h"
#include "Material.h"
#include "Mesh.h"
#include "Shader.h"
#include "components/RenderComponents.h"
#include "ShadowMap.h"
#include "MultisampleFramebuffer.h"

#include <glm/gtc/matrix_transform.hpp>
#include <glad/glad.h>

#include <cstdint>

Renderer::Renderer() = default;
Renderer::~Renderer() = default;

void Renderer::Init(int framebufferWidth, int framebufferHeight)
{
    // Depth test가 없으면 화면에 나중에 그린 triangle이 거리와 무관하게 앞에 표시된다.
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);

    // Main pass는 4x multisample target에 그리고 frame 끝에서 default framebuffer로 resolve한다.
    m_MSAAFramebuffer = std::make_unique<MultisampleFramebuffer>(
        framebufferWidth,
        framebufferHeight,
        4);

    // Shadow pass는 light 관점의 depth-only texture를 별도로 만든다.
    m_ShadowMap = std::make_unique<ShadowMap>(2048);
    m_ShadowShader = Shader::Create("shaders/ShadowDepth.glsl");

    m_LightDirection = glm::normalize(m_LightDirection);

    // 좌표: 작업공간 중심에서 광원 쪽으로 4m 이동한 가상 카메라가 중심을 바라봄.
    const glm::vec3 target{0.0F, 0.7F, 0.0F};
    const glm::vec3 lightPosition = target + m_LightDirection * 4.0F;

    const glm::mat4 lightView = glm::lookAt(
        lightPosition,
        target,
        glm::vec3{0.0F, 1.0F, 0.0F});

    // 범위: 광원 View의 가로·세로 각 5m, 앞뒤 깊이 0.1~10m. 원근 축소 없는 투영.
    const glm::mat4 lightProjection = glm::ortho(
        -2.5F, 2.5F,
        -2.5F, 2.5F,
        0.1F, 10.0F);

    m_LightSpaceMatrix = lightProjection * lightView;

    // 제한: 광원 범위와 그림자 해상도는 현재 작업공간에 맞춘 고정값.
}

void Renderer::BeginFrame()
{
    m_MSAAFramebuffer->Bind();

    glClearColor(0.14F, 0.15F, 0.16F, 1.0F);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
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

void Renderer::Draw(
    const glm::mat4& model,
    const MeshFilter& meshFilter,
    const MeshRenderer& meshRenderer,
    const glm::mat4& view,
    const glm::mat4& projection,
    const glm::vec3& cameraPosition)
{
    if (!meshFilter.mesh || !meshRenderer.shader || !meshRenderer.material) return;

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

    constexpr int baseColorTextureSlot = 0;
    if (meshRenderer.material->HasBaseColorTexture())
    {
        meshRenderer.material->GetBaseColorTexture()->Bind(baseColorTextureSlot);
        meshRenderer.shader->SetInt("u_BaseColorTexture", baseColorTextureSlot);
        meshRenderer.shader->SetInt("u_UseBaseColorTexture", 1);
    }
    else
    {
        meshRenderer.shader->SetInt("u_UseBaseColorTexture", 0);
    }

    meshRenderer.shader->SetFloat3("u_CameraPosition", cameraPosition);
    meshRenderer.shader->SetFloat3("u_LightDirection", m_LightDirection);
    meshRenderer.shader->SetMat4("u_LightSpaceMatrix", m_LightSpaceMatrix);

    // Shadow texture와 BaseColor texture가 충돌하지 않도록 고정 slot을 분리한다.
    constexpr int shadowTextureSlot = 7;
    m_ShadowMap->Bind(shadowTextureSlot);
    meshRenderer.shader->SetInt("u_ShadowMap", shadowTextureSlot);

    meshFilter.mesh->Bind();

    const std::size_t indexCount = meshFilter.indexCount == 0U
        ? meshFilter.mesh->GetIndexCount()
        : meshFilter.indexCount;

    // 단위: indexCount는 index 개수. 시작 index 번호를 EBO의 byte offset으로 변환.
    const std::size_t indexOffset = meshFilter.indexOffset * sizeof(std::uint32_t);

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
    m_ShadowShader->SetMat4("u_LightSpaceMatrix", m_LightSpaceMatrix);
}

void Renderer::DrawShadow(const glm::mat4& model, const MeshFilter& meshFilter)
{
    if (!meshFilter.mesh) return;

    // 이유: 그림자 판정에는 표면 색 없이 광원 기준 깊이만 필요.
    m_ShadowShader->SetMat4("u_Model", model);
    meshFilter.mesh->Bind();

    const std::size_t indexCount = meshFilter.indexCount == 0U
        ? meshFilter.mesh->GetIndexCount()
        : meshFilter.indexCount;
    const std::size_t indexOffset = meshFilter.indexOffset * sizeof(std::uint32_t);

    glDrawElements(
        GL_TRIANGLES,
        static_cast<GLsizei>(indexCount),
        GL_UNSIGNED_INT,
        reinterpret_cast<const void*>(indexOffset));

    meshFilter.mesh->UnBind();
}

void Renderer::EndShadowPass()
{
    m_ShadowShader->UnBind();
    m_ShadowMap->End();
}
