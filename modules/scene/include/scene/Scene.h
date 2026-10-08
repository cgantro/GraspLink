#pragma once

#include "scene/Entity.h"

#include <flecs.h>

#include <string>

namespace grasplink::scene
{

/**
 * @brief SceneRoot와 그 아래에 놓인 장면 Entity의 수명을 관리한다.
 * @details Flecs World가 Entity와 값을 소유하고 Scene은 root handle을 보관한다. Scene이 파괴되면 root와 자식 계층이 삭제되므로 물리 삭제 observer가 살아 있는 동안 World보다 먼저 파괴해야 한다.
 */
class Scene
{
public:
    /** @param world Scene보다 오래 살아 있어야 하는 Entity 저장소다. */
    explicit Scene(flecs::world& world);
    ~Scene();
    Scene(const Scene&) = delete;
    Scene& operator=(const Scene&) = delete;

    /** @brief Local/World 변환 값을 가진 Entity를 만들어 SceneRoot 아래에 연결한다. */
    Entity CreateEntity(const std::string& name = "");
    /** @brief Entity와 Component를 실제로 소유하는 Flecs World를 반환한다. */
    flecs::world& GetWorld();
    /** @brief Scene Entity가 연결된 최상위 부모의 Flecs handle을 반환한다. */
    flecs::entity GetSceneRoot() const;

private:
    flecs::world& m_World;
    flecs::entity m_SceneRoot{flecs::entity::null()};
};

}
