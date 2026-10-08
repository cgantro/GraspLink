#pragma once

#include <glm/vec3.hpp>

namespace grasplink::scene
{
class Entity;
}

namespace grasplink::simulation
{
/** @brief Entity에 정적 바닥 Body와 지정한 크기·중심 offset의 Box collider를 설정한다. */
void ConfigureStaticFloor(
    grasplink::scene::Entity& floor,
    const glm::vec3& halfExtentsMeters,
    const glm::vec3& localCenterOffsetMeters);
} // namespace grasplink::simulation
