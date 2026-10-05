#include "Material.h"

#include "Texture.h"

// Factor는 검증·보정하지 않고 저장한다. 허용 범위 제한은 shader의 metallic/roughness 계산에서 적용된다.
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
    // shared_ptr 대입으로 이전 texture 소유권을 놓고 새 참조를 공유한다.
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
