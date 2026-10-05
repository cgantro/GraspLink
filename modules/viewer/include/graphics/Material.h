#pragma once

#include <glm/glm.hpp>
#include <memory>

class Texture;

/**
 * @brief 표면의 기본색과 단순화된 금속성·거칠기 값을 보관한다.
 * @details
 * AssetManager가 asset 데이터로 Material을 만들고 Renderer가 값을 shader uniform으로 전달한다.
 * 기본색은 RGBA factor와 선택적 base color texture를 함께 둘 수 있다. Renderer는 둘을 곱해
 * 표면색을 만들며, metallic·roughness texture는 현재 연결하지 않고 계수만 전달한다.
 * Robot.glsl의 조명은 이 계수를 쓰는 단순화된 metallic/roughness 근사이며 완전한 glTF PBR은 아니다.
 * Texture는 shared_ptr로 공유하므로 마지막 참조가 사라질 때 GPU 자원이 해제된다. 그 시점까지
 * OpenGL context가 유효해야 한다.
 */
class Material final
{
public:
    /**
     * @brief 기본 표면 계수를 설정한다.
     * @param baseColorFactor RGBA 배율. RGB는 선형 색 값, alpha는 불투명도 배율이다.
     * @param metallicFactor 금속성 계수 [0..1], 단위 없음.
     * @param roughnessFactor 거칠기 계수 [0..1], 단위 없음.
     */
    Material(const glm::vec4& baseColorFactor, float metallicFactor, float roughnessFactor);

    const glm::vec4& BaseColorFactor() const;

    float MetallicFactor() const;

    float RoughnessFactor() const;

    /**
     * @brief 기본색 texture를 공유한다.
     * @param texture 연결할 GPU texture. nullptr이면 기존 texture를 제거한다.
     * @details Texture의 마지막 shared_ptr가 해제될 때 GPU 자원이 삭제되므로, 그때까지 GL context가 필요하다.
     */
    void SetBaseColorTexture(const std::shared_ptr<Texture>& texture);

    std::shared_ptr<Texture> GetBaseColorTexture() const;

    bool HasBaseColorTexture() const;

private:
    // (1,1,1,1)은 곱셈 항등값이므로 texture 색과 alpha를 그대로 통과시킨다.
    glm::vec4 m_BaseColorFactor{1.0F};

    // 0은 비금속, 1은 금속. Renderer와 Robot.glsl이 조명 근사에 사용한다.
    float m_MetallicFactor = 1.0F;

    // 0에 가까울수록 반사가 날카롭고 1에 가까울수록 넓게 퍼진다. shader는 최소값을 별도로 제한한다.
    float m_RoughnessFactor = 1.0F;

    // AssetManager가 공유한 GPU texture의 소유 참조. 마지막 소유자가 파괴될 때 Texture destructor가 GL ID를 삭제한다.
    std::shared_ptr<Texture> m_BaseColorTexture;
};
