#pragma once

#include <glm/glm.hpp>
#include <memory>
class Texture;
// 렌더러가 사용할 단순 GPU-side 색상/재질 값이다. 파일 포맷을 해석하지 않는다.
class Material final
{
public:
    Material(const glm::vec4& baseColorFactor,float metallicFactor,float roughnessFactor);

    const glm::vec4& BaseColorFactor() const;
    float MetallicFactor() const;
    float RoughnessFactor() const;

    void SetBaseColorTexture(const std::shared_ptr<Texture>& texture);
    std::shared_ptr<Texture> GetBaseColorTexture() const;
    bool HasBaseColorTexture() const;

private:
    glm::vec4 m_BaseColorFactor{1.0F};
    float m_MetallicFactor = 1.0F;
    float m_RoughnessFactor = 1.0F;

    std::shared_ptr<Texture> m_BaseColorTexture;
};
