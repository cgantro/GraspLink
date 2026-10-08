#pragma once

#include "application/PickPlaceScenarioSampler.h"

#include <memory>

namespace grasplink::scene
{
class Entity;
class Scene;
}

namespace grasplink::graphics
{
class Shader;
}

namespace grasplink::simulator::pick_place
{
/**
 * @brief 시뮬레이션 바닥에 움직이지 않는 충돌 형상을 설정한다.
 * @details 바닥은 Scene의 Entity로 표시되며 PhysicsSystemModule이 이를 Jolt의 Static Body로 만든다.
 */
void ConfigureFloor(grasplink::scene::Entity& floor);

/**
 * @brief Scene에 집기 대상 상자를 만들고 Entity handle을 반환한다.
 * @details 표시용 Mesh와 Jolt 충돌용 Box는 같은 크기를 사용한다.
 */
grasplink::scene::Entity CreateGraspBox(
    grasplink::scene::Scene& scene, const std::shared_ptr<grasplink::graphics::Shader>& shader);

/**
 * @brief 상자를 놓을 영역을 표시하는 Entity를 Scene에 만든다.
 * @details 이 영역은 배치 목표를 보여주는 표시용 Mesh이며 물리 Body는 만들지 않는다.
 */
grasplink::scene::Entity CreatePlacementArea(
    grasplink::scene::Scene& scene, const std::shared_ptr<grasplink::graphics::Shader>& shader);

/**
 * @brief 같은 scenario sampler에서 상자와 배치 영역의 위치·회전을 새로 뽑아 적용한다.
 * @details 위치와 회전은 각 Entity의 Local transform에 반영된다. 호출자는 이후 Scene의 World transform을 갱신해야 한다.
 */
void RandomizePickPlaceScene(
    grasplink::scene::Entity& box,
    grasplink::scene::Entity& placement,
    grasplink::application::PickPlaceScenarioSampler& sampler);
} // namespace grasplink::simulator::pick_place
