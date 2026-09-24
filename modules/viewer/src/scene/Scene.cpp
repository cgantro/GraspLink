#include "scene/Scene.h"

#include "components/TransformComponents.h"


Scene::Scene(flecs::world& world)
    : m_World(world)
{
}
/*
    ============================================================================
    CreateEntity
    ============================================================================

    Scene에 속하는 일반적인 공간 Entity를 생성한다.

    중요한 점은 Entity wrapper의 생성자가 아니라
    Scene::CreateEntity()에서 Transform Component를 붙인다는 것이다.


    왜 Entity 생성자에서 하지 않는가?

    Flecs Entity라고 해서 모두 3D 공간상의 물체인 것은 아니다.

    예:

        SceneManager 관련 Entity
        Global settings
        Runtime context
        Event entity

    등에는 Transform이 필요하지 않을 수 있다.

    따라서:

        Entity
            = 단순 Flecs handle wrapper

        Scene::CreateEntity()
            = 공간 Entity 생성

    으로 책임을 분리한다.
*/
Entity Scene::CreateEntity(const std::string& name)
{
    /*
        이름이 없으면 anonymous entity를 생성한다.

        이름이 있으면 Flecs debug / hierarchy 확인 시
        사람이 식별하기 쉽게 named entity를 생성한다.
    */
    flecs::entity entity =
        name.empty()
            ? m_World.entity()
            : m_World.entity(name.c_str());


    /*
        ------------------------------------------------------------------------
        Local Transform
        ------------------------------------------------------------------------

        Local은 부모 Entity 좌표계를 기준으로 한 값이다.

        예:

            Parent
             └─ Child

        Child Position(Local)이

            (0, 1, 0)

        이라는 것은 World의 (0, 1, 0)이 아니라

            "부모에서 Y축으로 1만큼 떨어진 위치"

        라는 뜻이다.
    */
    entity
        .set<Position, Local>(
            Position{0.0F, 0.0F, 0.0F})

        .set<Rotation, Local>(
            Rotation{0.0F, 0.0F, 0.0F})

        .set<Scale, Local>(
            Scale{1.0F})


        /*
            TransformSystem이 사용할 Matrix 저장 공간.

            Local Matrix:

                T * R * S

            World Matrix:

                ParentWorld * Local
        */
        .set<TransformMatrix, Local>(
            TransformMatrix{})

        .set<TransformMatrix, World>(
            TransformMatrix{});


    /*
        ------------------------------------------------------------------------
        Scene hierarchy
        ------------------------------------------------------------------------

        SceneRoot가 존재하면:

            Entity --ChildOf--> SceneRoot

        관계를 만든다.

        ChildOf 관계를 Flecs가 직접 관리하기 때문에
        Scene에서 별도의:

            std::vector<Entity*>

        같은 자식 목록을 유지할 필요가 없다.
    */
    if (m_SceneRoot.is_alive())
    {
        entity.child_of(m_SceneRoot);
    }


    /*
        실제 Component의 소유자는 Flecs World이고,
        Entity는 handle만 감싼 wrapper다.
    */
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
    /*
        SceneRoot는 Scene 단위 lifetime 관리용 Entity다.

        이 Entity 자체는 화면에 그려질 필요가 없으므로
        MeshFilter / MeshRenderer는 붙이지 않는다.

        또한 현재는 Scene 자체의 공간 Transform도 필요 없으므로
        Transform Component 역시 붙이지 않는다.

        SceneRoot는 단순 hierarchy root다.
    */
    m_SceneRoot =
        m_World.entity("SceneRoot");
}


void Scene::CleanupRoot()
{
    /*
        Scene이 이미 정리된 경우에는 아무 것도 하지 않는다.
    */
    if (!m_SceneRoot.is_alive())
    {
        return;
    }


    /*
        SceneRoot 제거.

        이 Scene에서 CreateEntity()로 만든 Entity들은
        SceneRoot 아래 ChildOf hierarchy에 속한다.

        Scene lifecycle을 Entity 각각에 대해 수동으로 관리하지 않고
        root 단위로 정리하기 위한 구조다.
    */
    m_SceneRoot.destruct();

    /*
        destruct된 Flecs Entity handle을 계속 보관하지 않도록
        명시적으로 null 상태로 돌려놓는다.
    */
    m_SceneRoot =
        flecs::entity::null();
}