#pragma once

#include <glm/glm.hpp>
#include <memory>

class Texture;

/**
 * @brief Renderer가 사용하는 GPU-side/runtime Material 상태를 보관한다.
 *
 * @details
 * Material은 GLB 파일을 직접 해석하지 않는다. GltfLoader가 MaterialData를 만들고,
 * AssetManager가 이를 이 객체로 변환한다. 현재 렌더 경로는 metallic-roughness factor와
 * base-color texture를 중심으로 사용한다.
 *
 * PBR factor는 모두 무차원 값이다. baseColorFactor는 RGBA 계수이며 metallic/roughness는
 * 일반적으로 0..1 범위를 사용한다. Texture pixel 색공간 처리는 Texture 생성 정책에서 결정한다.
 *
 * @todo [FUTURE] normal, metallicRoughness, occlusion, emissive texture와 alphaMode를 추가한다.
 */
class Material final
{
public:
    /**
     * @brief PBR 기본 factor로 Material을 생성한다.
     * @param baseColorFactor RGBA multiplier, 무차원.
     * @param metallicFactor 금속성 계수. 일반적으로 0..1.
     * @param roughnessFactor 거칠기 계수. 일반적으로 0..1.
     */
    Material(const glm::vec4& baseColorFactor, float metallicFactor, float roughnessFactor);

    /** @return Base color RGBA multiplier reference. */
    const glm::vec4& BaseColorFactor() const;

    /** @return Metallic factor. 무차원. */
    float MetallicFactor() const;

    /** @return Roughness factor. 무차원. */
    float RoughnessFactor() const;

    /**
     * @brief Base-color Texture의 shared ownership을 연결한다.
     * @param texture 연결할 GPU Texture. nullptr로 texture를 제거할 수 있다.
     */
    void SetBaseColorTexture(const std::shared_ptr<Texture>& texture);

    /** @return 연결된 Base-color Texture shared_ptr. 없으면 nullptr. */
    std::shared_ptr<Texture> GetBaseColorTexture() const;

    /** @return Base-color Texture가 연결되어 있으면 true. */
    bool HasBaseColorTexture() const;

private:
    /** @brief Shader base-color에 곱할 RGBA factor. */
    glm::vec4 m_BaseColorFactor{1.0F};

    /** @brief PBR metallic factor, 무차원. */
    float m_MetallicFactor = 1.0F;

    /** @brief PBR roughness factor, 무차원. */
    float m_RoughnessFactor = 1.0F;

    /** @brief 선택적인 base-color GPU Texture shared ownership. */
    std::shared_ptr<Texture> m_BaseColorTexture;
};
