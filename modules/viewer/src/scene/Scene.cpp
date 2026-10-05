#include "scene/Scene.h"

#include "components/TransformComponents.h"
#include <stdexcept>


Scene::Scene(flecs::world& world)
    : m_World(world)
{
}
Entity Scene::CreateEntity(const std::string& name)
{
    if (!m_SceneRoot.is_alive())
        throw std::logic_error("Scene::CreateEntity requires an active Scene; create Entities in OnEnter");
    flecs::entity entity =
        name.empty()
            ? m_World.entity()
            : m_World.entity(name.c_str());


    // 좌표: 부모 기준 Local. 기본 위치·회전은 원점, 크기 배율은 단위값이다.
    entity
        .set<Position, Local>(
            Position{0.0F, 0.0F, 0.0F})

        .set<Rotation, Local>(
            Rotation{})

        .set<Scale, Local>(
            Scale{1.0F})


        // TransformSystemModule이 Local 값과 누적 World 행렬을 갱신할 저장 공간.
        .set<TransformMatrix, Local>(
            TransformMatrix{})

        .set<TransformMatrix, World>(
            TransformMatrix{});


    entity.child_of(m_SceneRoot);

    return Entity{entity};
}


flecs::world& Scene::GetWorld()
{
    return m_World;
}


flecs::entity Scene::GetSceneRoot() const
{
    return m_SceneRoot;
}


void Scene::InitRoot()
{
    // root는 계층 경계다. Local TRS 없이 자손의 기준 World 행렬을 둔다.
    m_SceneRoot =
        m_World.entity("SceneRoot").add<SceneRootTag>();
}


void Scene::CleanupRoot()
{
    if (!m_SceneRoot.is_alive())
    {
        return;
    }


    // Flecs가 ChildOf 자손과 Component를 함께 제거한다. 외부 handle은 무효가 된다.
    m_SceneRoot.destruct();

    m_SceneRoot =
        flecs::entity::null();
}
