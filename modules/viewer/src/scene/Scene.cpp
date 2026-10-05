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


    // 부모 기준 TRS 기본값: 위치·회전은 0, 크기 배율은 1.
    entity
        .set<Position, Local>(
            Position{0.0F, 0.0F, 0.0F})

        .set<Rotation, Local>(
            Rotation{0.0F, 0.0F, 0.0F})

        .set<Scale, Local>(
            Scale{1.0F})


        // TransformSystemModule이 갱신할 Local/World 행렬 저장 공간.
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
    // SceneRoot에는 Local TRS를 붙이지 않는다. TransformSystem이 유도한 World 행렬을 캐시한다.
    m_SceneRoot =
        m_World.entity("SceneRoot").add<SceneRootTag>();
}


void Scene::CleanupRoot()
{
    if (!m_SceneRoot.is_alive())
    {
        return;
    }


    // root와 모든 자식을 삭제한 뒤 handle을 비워 다음 생성 요청을 차단한다.
    m_SceneRoot.destruct();

    m_SceneRoot =
        flecs::entity::null();
}
