#include "Material.h"

#include "Texture.h"

/*
    Material은 "파일 형식"도 "Shader 코드"도 아니다.
    렌더링 시 Shader에 전달할 표면 파라미터를 보관하는 runtime 데이터 객체다.

    baseColorFactor : 표면의 기본 RGBA 배율
    metallicFactor  : 0=비금속, 1=금속에 가까운 PBR 계수
    roughnessFactor : 0=매끈한 반사, 1=거친 반사에 가까운 계수
*/
Material::Material(
    const glm::vec4& baseColorFactor,
    float metallicFactor,
    float roughnessFactor)
    : m_BaseColorFactor(baseColorFactor),
      m_MetallicFactor(metallicFactor),
      m_RoughnessFactor(roughnessFactor)
{
}

const glm::vec4& Material::BaseColorFactor() const
{
    return m_BaseColorFactor;
}

float Material::MetallicFactor() const
{
    return m_MetallicFactor;
}

float Material::RoughnessFactor() const
{
    return m_RoughnessFactor;
}

void Material::SetBaseColorTexture(const std::shared_ptr<Texture>& texture)
{
    // shared_ptr를 보관해 AssetManager와 Material이 같은 GPU Texture lifetime을 안전하게 공유한다.
    m_BaseColorTexture = texture;
}

std::shared_ptr<Texture> Material::GetBaseColorTexture() const
{
    return m_BaseColorTexture;
}

bool Material::HasBaseColorTexture() const
{
    return m_BaseColorTexture != nullptr;
}
