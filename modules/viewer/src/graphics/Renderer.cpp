#include "Renderer.h"

#include "Texture.h"
#include "Material.h"
#include "Mesh.h"
#include "Shader.h"
#include "components/RenderComponents.h"
#include "MultisampleFramebuffer.h"

#include <glad/glad.h>

#include <cstdint>
#include <algorithm>

Renderer::Renderer() = default;
Renderer::~Renderer() = default;

void Renderer::Init(int framebufferWidth, int framebufferHeight)
{
    // 깊이 검사를 끄면 멀리 있는 삼각형도 그린 순서가 나중이라는 이유만으로 가까운 물체 앞에 표시된다.
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);

    // 화면 그리기 단계(Main pass)는 픽셀마다 네 표본을 가진 대상을 사용한다. 프레임 끝에 표본을 합쳐 Window 기본 framebuffer로 복사한다.
    m_MSAAFramebuffer = std::make_unique<MultisampleFramebuffer>(
        framebufferWidth,
        framebufferHeight,
        4);

    m_LightDirection = glm::normalize(m_LightDirection);
}

void Renderer::BeginFrame()
{
    glDisable(GL_SCISSOR_TEST);
    m_MSAAFramebuffer->Bind();

    glClearColor(0.14F, 0.15F, 0.16F, 1.0F);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::EndFrame()
{
    glDisable(GL_SCISSOR_TEST);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, m_MSAAFramebuffer->GetWidth(), m_MSAAFramebuffer->GetHeight());
    glClearColor(0.14F, 0.15F, 0.16F, 1.0F);
    glClear(GL_COLOR_BUFFER_BIT);
    m_MSAAFramebuffer->ResolveToDefault(
        m_SceneViewport.x, m_SceneViewport.y,
        m_SceneViewport.z, m_SceneViewport.w);
}

void Renderer::Resize(int width, int height)
{
    if (m_MSAAFramebuffer)
    {
        m_MSAAFramebuffer->Resize(width, height);
        m_SceneViewport = {0, 0, width, height};
    }
}

void Renderer::SetSceneViewport(int x, int y, int width, int height)
{
    if (!m_MSAAFramebuffer || width <= 0 || height <= 0)
        return;

    const int left = std::clamp(x, 0, m_MSAAFramebuffer->GetWidth());
    const int bottom = std::clamp(y, 0, m_MSAAFramebuffer->GetHeight());
    const int right = std::clamp(x + width, left, m_MSAAFramebuffer->GetWidth());
    const int top = std::clamp(y + height, bottom, m_MSAAFramebuffer->GetHeight());
    m_SceneViewport = {left, bottom, right - left, top - bottom};
}

void Renderer::Draw(
    const glm::mat4& model,
    const MeshFilter& meshFilter,
    const MeshRenderer& meshRenderer,
    const glm::mat4& view,
    const glm::mat4& projection,
    const glm::vec3& cameraPosition)
{
    if (m_SceneViewport.z > 0 && m_SceneViewport.w > 0)
    {
        glViewport(m_SceneViewport.x, m_SceneViewport.y, m_SceneViewport.z, m_SceneViewport.w);
        glEnable(GL_SCISSOR_TEST);
        glScissor(m_SceneViewport.x, m_SceneViewport.y, m_SceneViewport.z, m_SceneViewport.w);
    }

    // Mesh나 재질처럼 필요한 렌더 자료가 빠진 Entity는 GPU 상태를 바꾸기 전에 건너뛴다.
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

    meshFilter.mesh->Bind();

    const std::size_t indexCount = meshFilter.indexCount == 0U
        ? meshFilter.mesh->GetIndexCount()
        : meshFilter.indexCount;

    // indexCount는 byte 수가 아니라 index 원소 개수다. 시작 원소 번호는 GPU buffer가 요구하는 byte offset으로 바꾼다.
    const std::size_t indexOffset = meshFilter.indexOffset * sizeof(std::uint32_t);

    glDrawElements(
        GL_TRIANGLES,
        static_cast<GLsizei>(indexCount),
        GL_UNSIGNED_INT,
        reinterpret_cast<const void*>(indexOffset));

    meshFilter.mesh->UnBind();
    meshRenderer.shader->UnBind();

}
