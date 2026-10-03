#pragma once

#include <glm/glm.hpp>
#include <memory>

class Texture;

/**
 * @brief Renderer가 사용하는 런타임 Material 상태를 보관한다.
 *
 * @details
 * Material은 GLB 파일을 직접 해석하지 않는다. GltfLoader가 MaterialData를 만들고,
 * AssetManager가 이를 GPU-side Material로 변환한다. 현재는 metallic-roughness factor와
 * base color texture만 실제 렌더링 경로에 연결되어 있다.
 *
 * @todo [FUTURE] normal, metallicRoughness, occlusion, emissive texture와 alphaMode를 추가한다.
 */
class Material final
{
public:
    /** @brief PBR 기본 factor로 Material을 생성한다. */
    Material(const glm::vec4& baseColorFactor, float metallicFactor, float roughnessFactor);

    /** @brief Base color RGBA factor를 반환한다. */
    const glm::vec4& BaseColorFactor() const;

    /** @brief Metallic factor를 반환한다. */
    float MetallicFactor() const;

    /** @brief Roughness factor를 반환한다. */
    float RoughnessFactor() const;

    /** @brief Base color Texture의 shared ownership을 연결한다. */
    void SetBaseColorTexture(const std::shared_ptr<Texture>& texture);

    /** @brief 연결된 Base color Texture를 반환한다. */
    std::shared_ptr<Texture> GetBaseColorTexture() const;

    /** @brief Base color Texture 존재 여부를 반환한다. */
    bool HasBaseColorTexture() const;

private:
    glm::vec4 m_BaseColorFactor{1.0F};
    float m_MetallicFactor = 1.0F;
    float m_RoughnessFactor = 1.0F;
    std::shared_ptr<Texture> m_BaseColorTexture;
};
