#include "Entity.h"

#include "components/TransformComponents.h"
#include <stdexcept>


Entity::Entity(flecs::entity handle)
    : m_EntityHandle(handle)
{
}


// ============================================================================
// 변환
// ============================================================================

glm::vec3 Entity::GetLocalPosition() const
{
    // Entity wrapper는 소유권 없는 handle이므로, 파괴 뒤 접근은 기본값으로 종료한다.
    if (!IsValid())
        return glm::vec3(0.0f);

    if (!m_EntityHandle.has<Position, Local>())
        return glm::vec3(0.0f);

    return m_EntityHandle.get<Position, Local>();
}


glm::quat Entity::GetLocalRotation() const
{
    if (!IsValid())
        return glm::quat(1.0F, 0.0F, 0.0F, 0.0F);

    if (!m_EntityHandle.has<Rotation, Local>())
        return glm::quat(1.0F, 0.0F, 0.0F, 0.0F);

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

    // 저장소에는 부모 기준 값만 쓴다. World 행렬 계산은 TransformSystem의 다음 갱신에 맡긴다.
    m_EntityHandle.set<Position, Local>(
        Position{position}
    );
}


void Entity::SetLocalRotation(const glm::quat& rotation)
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

    // 파생 행렬을 getter에서 즉석 계산하지 않아 모든 consumer가 한 번의 시스템 갱신 결과를 공유한다.
    if (!m_EntityHandle.has<TransformMatrix, World>())
        return glm::mat4(1.0f);

    return m_EntityHandle.get<TransformMatrix, World>();
}


// ============================================================================
// 계층
// ============================================================================

Entity& Entity::SetParent(const Entity& parent)
{
    if (!IsValid() || !parent.IsValid())
        return *this;

    // Flecs 관계는 같은 World 내부에서만 안전하게 연결할 수 있다.
    if (m_EntityHandle.world().c_ptr() != parent.GetHandle().world().c_ptr())
        throw std::invalid_argument("Entity::SetParent requires the same Flecs World");
    // SceneRootTag까지 올라가 Scene 소속을 찾는다. 일반 grouping node는 소유 경계가 아니며,
    // 이미 Scene에 속한 Entity를 다른 Scene이나 외부 계층으로 빼지 못하게 한다.
    auto sceneRoot = [](flecs::entity handle)
    {
        while (handle.id() != 0 && handle.is_alive())
        {
            if (handle.has<SceneRootTag>()) return handle;
            handle = handle.parent();
        }
        return flecs::entity::null();
    };
    const flecs::entity owner = sceneRoot(m_EntityHandle);
    if (owner.id() != 0 && owner != sceneRoot(parent.GetHandle()))
        throw std::invalid_argument("Entity::SetParent cannot leave the owning Scene");
    // 부모 후보의 조상에 자기 자신이 있으면 새 ChildOf 관계가 순환을 만들어 World 갱신이 끝나지 않는다.
    for (flecs::entity ancestor = parent.GetHandle(); ancestor.id() != 0 && ancestor.is_alive(); ancestor = ancestor.parent())
        if (ancestor == m_EntityHandle)
            throw std::invalid_argument("Entity::SetParent cannot create a hierarchy cycle");

    // Local TRS를 보존하고 관계만 바꾼다. 따라서 World 자세 보존이 필요하면 호출자가 Local 값을 다시 계산한다.
    m_EntityHandle.child_of(
        parent.GetHandle()
    );

    return *this;
}


Entity& Entity::AddChild(const Entity& child)
{
    if (!IsValid() || !child.IsValid())
        return *this;

    // 검증·Scene 경계·순환 검사를 SetParent 한 곳에 유지한다.
    Entity childHandle = child;
    childHandle.SetParent(*this);

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

    // Flecs lookup은 이 Entity를 기준으로 이름 경로를 해석한다. 없는 경로는 무효 wrapper로 통일한다.
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

    // 현재 handle은 검사하지 않고 자식부터 깊이 우선 탐색한다. 이름 없는 grouping node도 재귀로 통과한다.
    flecs::entity found =
        flecs::entity::null();


    m_EntityHandle.children(
        [&](flecs::entity child)
        {
            // 이미 첫 일치를 찾았으면 나머지 형제 subtree는 읽지 않는다.
            if (found != flecs::entity::null())
                return;

            const char* childName = child.name().c_str();
            if (childName && targetName == childName)
            {
                found = child;
                return;
            }

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
// 보조 기능
// ============================================================================

bool Entity::IsValid() const
{
    // wrapper 복사본 자체가 소멸 여부를 추적하지 않고 Flecs World의 live 상태를 질의한다.
    return m_EntityHandle.is_alive();
}


void Entity::Destroy()
{
    if (!IsValid())
        return;

    // Flecs가 Entity와 ChildOf hierarchy를 정리한다. 다른 wrapper는 남지만 이후 IsValid()==false다.
    m_EntityHandle.destruct();
}
