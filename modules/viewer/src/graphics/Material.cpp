#include "graphics/Material.h"

#include "graphics/Texture.h"

namespace grasplink::graphics
{

// Material은 전달된 Factor 값을 그대로 저장한다. 값의 유효 범위 제한은 Metallic/Roughness를 해석하는 Shader가 적용한다.
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
    // 새 Texture의 공유 참조를 저장한다. 대입과 함께 이전 Texture에 대한 공유 참조는 놓인다.
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

} // namespace grasplink::graphics
