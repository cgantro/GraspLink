#include "Entity.h"

#include "components/TransformComponents.h"


Entity::Entity(flecs::entity handle)
    : m_EntityHandle(handle)
{
}


// ============================================================================
// Transform
// ============================================================================

glm::vec3 Entity::GetLocalPosition() const
{
    /*
        Flecs Pair:
            (Position, Local)
        을 읽는다.

        World Position을 별도로 저장하지 않는 이유는
        최종 World 위치를 TransformMatrix(World)에서 얻을 수 있기 때문이다.
    */
    if (!IsValid())
        return glm::vec3(0.0f);

    if (!m_EntityHandle.has<Position, Local>())
        return glm::vec3(0.0f);

    return m_EntityHandle.get<Position, Local>();
}


glm::vec3 Entity::GetLocalRotation() const
{
    if (!IsValid())
        return glm::vec3(0.0f);

    if (!m_EntityHandle.has<Rotation, Local>())
        return glm::vec3(0.0f);

    return m_EntityHandle.get<Rotation, Local>();
}


glm::vec3 Entity::GetLocalScale() const
{
    if (!IsValid())
        return glm::vec3(1.0f);

    if (!m_EntityHandle.has<Scale, Local>())
        return glm::vec3(1.0f);

    return m_EntityHandle.get<Scale, Local>();
}


void Entity::SetLocalPosition(const glm::vec3& position)
{
    if (!IsValid())
        return;

    /*
        (Position, Local) Component를 수정한다.

        이후 TransformSystem 실행 시:

            Position(Local)
              ↓
            TransformMatrix(Local)
              ↓
            TransformMatrix(World)

        순으로 반영된다.
    */
    m_EntityHandle.set<Position, Local>(
        Position{position}
    );
}


void Entity::SetLocalRotation(const glm::vec3& rotation)
{
    if (!IsValid())
        return;

    m_EntityHandle.set<Rotation, Local>(
        Rotation{rotation}
    );
}


void Entity::SetLocalScale(const glm::vec3& scale)
{
    if (!IsValid())
        return;

    m_EntityHandle.set<Scale, Local>(
        Scale{scale}
    );
}


glm::mat4 Entity::GetWorldMatrix() const
{
    if (!IsValid())
        return glm::mat4(1.0f);

    /*
        TransformSystem이 계산해 둔

            (TransformMatrix, World)

        Component를 조회한다.

        Entity 자체는 World Matrix를 계산하지 않는다.

        이것이 ECS의 중요한 책임 분리다.

            Entity
                -> 접근 인터페이스

            TransformSystem
                -> 실제 계산
    */
    if (!m_EntityHandle.has<TransformMatrix, World>())
        return glm::mat4(1.0f);

    return m_EntityHandle.get<TransformMatrix, World>();
}


// ============================================================================
// Hierarchy
// ============================================================================

Entity& Entity::SetParent(const Entity& parent)
{
    if (!IsValid() || !parent.IsValid())
        return *this;

    /*
        Flecs ChildOf Relation 생성.

            child.child_of(parent)

        이후 Flecs가 이 Entity hierarchy를 알고 있으므로

            parent()
            children()
            cascade()

        등을 사용할 수 있다.
    */
    m_EntityHandle.child_of(
        parent.GetHandle()
    );

    return *this;
}


Entity& Entity::AddChild(const Entity& child)
{
    if (!IsValid() || !child.IsValid())
        return *this;

    /*
        실제 관계 방향은 항상:

            Child --ChildOf--> Parent

        이다.

        AddChild는 편의를 위한 wrapper일 뿐이다.
    */
    child.GetHandle().child_of(
        m_EntityHandle
    );

    return *this;
}


Entity Entity::GetParent() const
{
    if (!IsValid())
        return Entity{};

    return Entity{
        m_EntityHandle.parent()
    };
}


std::vector<Entity> Entity::GetChildren() const
{
    std::vector<Entity> children;

    if (!IsValid())
        return children;

    /*
        Flecs가 관리하는 ChildOf 관계를 이용해서
        직속 자식 Entity들을 순회한다.

        별도의 vector<Entity*> children을 직접 관리하지 않는다.
    */
    m_EntityHandle.children(
        [&](flecs::entity child)
        {
            children.emplace_back(child);
        }
    );

    return children;
}


Entity Entity::GetChild(const std::string& name) const
{
    if (!IsValid())
        return Entity{};

    /*
        현재 Entity 아래에서 해당 이름의 Child를 찾는다.

        이 함수는 직접적인 path/child 검색 용도이고,
        깊은 hierarchy 전체 검색은 아래 Recursive 함수를 사용한다.
    */
    flecs::entity child =
        m_EntityHandle.lookup(
            name.c_str()
        );

    if (!child.is_alive())
        return Entity{};

    return Entity{child};
}


Entity Entity::FindChildByNameRecursive(
    const std::string& targetName
) const
{
    if (!IsValid())
        return Entity{};

    flecs::entity found =
        flecs::entity::null();


    /*
        Depth First Search.

        예:

            RobotRoot
             └─ Base
                 └─ Joint1
                     └─ Link1
                         └─ Joint2


        targetName = "Joint2"

        라면:

            Base
             ↓
            Joint1
             ↓
            Link1
             ↓
            Joint2

        순서로 하위 hierarchy를 탐색한다.


        이후 GLB Node 이름을 이용해:

            J1
            J2
            J3
            ...
            EE

        를 찾는 데 사용할 수 있다.
    */
    m_EntityHandle.children(
        [&](flecs::entity child)
        {
            /*
                이미 찾았다면 나머지 순회에서는 아무 것도 하지 않는다.
            */
            if (found != flecs::entity::null())
                return;


            /*
                현재 Child 이름 검사.
            */
            if (std::string(child.name().c_str()) == targetName)
            {
                found = child;
                return;
            }


            /*
                현재 Child의 하위 hierarchy를 재귀 탐색한다.
            */
            Entity childEntity{child};

            Entity result =
                childEntity.FindChildByNameRecursive(
                    targetName
                );

            if (result.IsValid())
            {
                found = result.GetHandle();
            }
        }
    );


    if (found == flecs::entity::null())
        return Entity{};

    return Entity{found};
}


// ============================================================================
// Utility
// ============================================================================

bool Entity::IsValid() const
{
    /*
        is_alive():

        해당 Entity ID가 현재 Flecs World에서 실제로 살아 있는지 확인한다.

        단순히 handle 값이 존재하는지를 검사하는 것보다
        Entity가 destruct된 경우까지 확인할 수 있다.
    */
    return m_EntityHandle.is_alive();
}


void Entity::Destroy()
{
    if (!IsValid())
        return;

    /*
        Flecs World에서 Entity를 제거한다.

        ChildOf 관계가 있는 Entity 제거 정책은 Flecs hierarchy 설정과
        관계 semantics에 따라 영향을 받으므로,
        이후 SceneRoot 단위 정리를 만들 때 다시 확인한다.
    */
    m_EntityHandle.destruct();
}
