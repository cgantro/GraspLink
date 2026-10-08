#pragma once

#include <cstddef>
#include <cstdint>

namespace grasplink::simulation::robotics
{

enum class SelfCollisionPart : std::uint8_t
{
    Base,
    Link,
    GripperBody
};

struct SelfCollisionIdentity
{
    SelfCollisionPart part = SelfCollisionPart::Link;
    std::size_t linkIndex = 0;
};

/**
 * @brief HCR-12A 자가 충돌 검사에서 실제로 맞닿도록 설계된 부품 쌍을 허용한다.
 * @details 같은 링크의 여러 조각은 하나의 Collider에 묶인다. 이 표는 이웃 링크, Base와 Link1 베어링, J6와 그리퍼 몸체의 연결부만 허용한다.
 */
[[nodiscard]] constexpr bool IsAllowedSelfCollision(
    SelfCollisionIdentity first,
    SelfCollisionIdentity second) noexcept
{
    if (first.part == SelfCollisionPart::Link && second.part == SelfCollisionPart::Link)
    {
        constexpr std::size_t invalidIndex = static_cast<std::size_t>(-1);
        if (first.linkIndex == invalidIndex || second.linkIndex == invalidIndex)
            return false;
        const std::size_t distance = first.linkIndex > second.linkIndex
            ? first.linkIndex - second.linkIndex
            : second.linkIndex - first.linkIndex;
        return distance <= 1;
    }

    if (first.part == SelfCollisionPart::Base && second.part == SelfCollisionPart::Link)
        return second.linkIndex == 0;
    if (second.part == SelfCollisionPart::Base && first.part == SelfCollisionPart::Link)
        return first.linkIndex == 0;

    if (first.part == SelfCollisionPart::Link && second.part == SelfCollisionPart::GripperBody)
        return first.linkIndex == 5;
    if (second.part == SelfCollisionPart::Link && first.part == SelfCollisionPart::GripperBody)
        return second.linkIndex == 5;

    return false;
}

} // namespace grasplink::simulation::robotics
