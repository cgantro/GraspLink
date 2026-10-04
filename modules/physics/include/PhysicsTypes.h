#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdint>

namespace grasplink::physics
{

struct PhysicsBodyHandle
{
    static constexpr std::uint32_t InvalidValue = 0xFFFFFFFFU;

    std::uint32_t value = InvalidValue;

    [[nodiscard]]
    bool IsValid() const noexcept
    {
        return value != InvalidValue;
    }
};


/*
 * Physics Body의 위치와 회전.
 *
 * GLM을 공용 Math 타입으로 사용한다.
 *
 * position:
 *   World 좌표계 기준 위치 [m]
 *
 * rotation:
 *   World 좌표계 기준 회전 Quaternion
 *
 * glm::quat 기본 생성 순서는 (w, x, y, z)다.
 */
struct Transform
{
    glm::vec3 position{0.0F};

    glm::quat rotation{
        1.0F,
        0.0F,
        0.0F,
        0.0F
    };
};


enum class BodyMotionType : std::uint8_t
{
    Static,
    Kinematic,
    Dynamic
};


struct BoxBodyDescription
{
    /*
     * Box의 전체 크기가 아니라 반쪽 크기.
     *
     * {0.5, 0.5, 0.5}
     * -> 실제 크기 1m x 1m x 1m
     */
    glm::vec3 halfExtentsMeters{
        0.5F,
        0.5F,
        0.5F
    };

    Transform transform;

    BodyMotionType motionType =
        BodyMotionType::Dynamic;
};

} // namespace grasplink::physics