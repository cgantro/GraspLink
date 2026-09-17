#pragma once

#include <glm/glm.hpp>

namespace PoseLink
{
// 렌더러가 사용할 단순 GPU-side 색상/재질 값이다. 파일 포맷을 해석하지 않는다.
class Material final
{
public:
    Material(glm::vec4 baseColorFactor, float metallicFactor = 0.0F,
             float roughnessFactor = 1.0F)
        : baseColorFactor_(baseColorFactor), metallicFactor_(metallicFactor),
          roughnessFactor_(roughnessFactor) {}

    const glm::vec4& BaseColorFactor() const noexcept { return baseColorFactor_; }
    float MetallicFactor() const noexcept { return metallicFactor_; }
    float RoughnessFactor() const noexcept { return roughnessFactor_; }

private:
    glm::vec4 baseColorFactor_;
    float metallicFactor_;
    float roughnessFactor_;
};
}
