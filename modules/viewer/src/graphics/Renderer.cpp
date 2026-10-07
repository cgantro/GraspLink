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

    // 그림자 단계는 광원에서 본 표면의 투영 깊이를 별도 Texture에 저장한다. 투영 깊이는 실제 광원까지의 직선거리가 아니다.
    m_ShadowMap = std::make_unique<ShadowMap>(2048);
    m_ShadowShader = Shader::Create("shaders/ShadowDepth.glsl");

    m_LightDirection = glm::normalize(m_LightDirection);

    const glm::vec3 target{0.0F, 0.7F, 0.0F};
    // 그림자 카메라는 작업 공간 중심에서 광원 방향으로 4 m 떨어진 위치에서 중심을 바라본다.
    const glm::vec3 lightPosition = target + m_LightDirection * 4.0F;

    const glm::mat4 lightView = glm::lookAt(
        lightPosition,
        target,
        glm::vec3{0.0F, 1.0F, 0.0F});

    // 광원 시야는 가로와 세로 각각 5 m이고 앞뒤 거리는 0.1~10 m다. 원근에 따라 크기가 달라지지 않는 직교 투영을 쓴다.
    const glm::mat4 lightProjection = glm::ortho(
        -2.5F, 2.5F,
        -2.5F, 2.5F,
        0.1F, 10.0F);

    m_LightSpaceMatrix = lightProjection * lightView;

    // 광원이 비추는 범위와 깊이 Texture 해상도는 현재 작업 공간에 맞춰 코드에 고정되어 있다.
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
    meshRenderer.shader->SetMat4("u_LightSpaceMatrix", m_LightSpaceMatrix);

    // 그림자 Texture와 표면 기본색 Texture가 같은 위치를 덮어쓰지 않도록 서로 다른 Texture unit에 연결한다.
    constexpr int shadowTextureSlot = 7;
    m_ShadowMap->Bind(shadowTextureSlot);
    meshRenderer.shader->SetInt("u_ShadowMap", shadowTextureSlot);

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

void Renderer::BeginShadowPass()
{
    m_ShadowMap->Begin();
    m_ShadowShader->Bind();
    m_ShadowShader->SetMat4("u_LightSpaceMatrix", m_LightSpaceMatrix);
}

void Renderer::DrawShadow(const glm::mat4& model, const MeshFilter& meshFilter)
{
    if (!meshFilter.mesh) return;

    // 그림자 계산 단계에서는 표면 색이 필요 없고 광원에서 본 깊이만 저장하면 된다.
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
