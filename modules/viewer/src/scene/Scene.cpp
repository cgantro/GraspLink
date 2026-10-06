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


    // 새 Entity의 위치와 회전은 부모 기준 원점에서 시작하고, 크기 배율은 축마다 1인 상태로 시작한다.
    entity
        .set<Position, Local>(
            Position{0.0F, 0.0F, 0.0F})

        .set<Rotation, Local>(
            Rotation{})

        .set<Scale, Local>(
            Scale{1.0F})


        // TransformSystemModule이 Entity 자체 행렬과 조상 변환을 누적한 Scene 행렬을 저장할 공간을 미리 붙인다.
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
    // root는 Scene 소속 자손을 묶는 부모다. 자체 위치·회전·크기(Local TRS)는 없고, 자손이 누적 변환을 시작할 항등 Scene 행렬만 가진다.
    m_SceneRoot =
        m_World.entity("SceneRoot").add<SceneRootTag>();
}


void Scene::CleanupRoot()
{
    if (!m_SceneRoot.is_alive())
    {
        return;
    }


    // Flecs는 ChildOf 관계로 연결된 모든 자손 Entity와 Component를 함께 제거한다. 이 계층을 가리키던 외부 handle도 더는 유효하지 않다.
    m_SceneRoot.destruct();

    m_SceneRoot =
        flecs::entity::null();
}
