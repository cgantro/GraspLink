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
/*
 * [추가 그래픽스 용어 설명]
 * - Material: Mesh 표면이 빛과 어떻게 상호작용할지 정하는 값 묶음.
 * - PBR(Physically Based Rendering): 금속성/거칠기 같은 물리적 특성을 이용해 빛 반응을 계산하는 렌더링 방식.
 * - Base Color / Albedo: 조명 효과를 제외한 표면의 기본 색.
 * - Metallic: 표면이 금속처럼 반사되는 정도. 일반적으로 0=비금속, 1=금속.
 * - Roughness: 표면 거칠기. 0에 가까울수록 반사가 날카롭고, 1에 가까울수록 넓게 퍼진다.
 * - Texture: 표면의 위치마다 다른 색/값을 제공하는 2D 이미지 데이터.
 * - Factor: Texture 결과에 곱해 최종 값을 조정하는 무차원 계수.
 *
 * baseColorFactor는 RGBA 순서이며 각 성분은 보통 0..1 범위를 사용한다.
 * metallicFactor/roughnessFactor도 무차원 값이다.
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
    // RGBA 기본색 factor. (1,1,1,1)은 texture/기본색을 그대로 사용하는 흰색 배율이다.
    glm::vec4 m_BaseColorFactor{1.0F};

    // 금속성 계수. 무차원.
    float m_MetallicFactor = 1.0F;

    // 거칠기 계수. 무차원.
    float m_RoughnessFactor = 1.0F;

    // 여러 Material/Asset이 같은 GPU Texture를 공유할 수 있어 shared_ptr를 사용한다.
    std::shared_ptr<Texture> m_BaseColorTexture;
};
