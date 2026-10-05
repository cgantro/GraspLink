#include "Entity.h"

#include "components/TransformComponents.h"
#include <stdexcept>


Entity::Entity(flecs::entity handle)
    : m_EntityHandle(handle)
{
}


// ============================================================================
// Transform
// ============================================================================

glm::vec3 Entity::GetLocalPosition() const
{
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

    if (m_EntityHandle.world().c_ptr() != parent.GetHandle().world().c_ptr())
        throw std::invalid_argument("Entity::SetParent requires the same Flecs World");
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
    for (flecs::entity ancestor = parent.GetHandle(); ancestor.id() != 0 && ancestor.is_alive(); ancestor = ancestor.parent())
        if (ancestor == m_EntityHandle)
            throw std::invalid_argument("Entity::SetParent cannot create a hierarchy cycle");

    m_EntityHandle.child_of(
        parent.GetHandle()
    );

    return *this;
}


Entity& Entity::AddChild(const Entity& child)
{
    if (!IsValid() || !child.IsValid())
        return *this;

    // 계층 변경 규칙은 SetParent에 위임한다.
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


    m_EntityHandle.children(
        [&](flecs::entity child)
        {
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
// Utility
// ============================================================================

bool Entity::IsValid() const
{
    return m_EntityHandle.is_alive();
}


void Entity::Destroy()
{
    if (!IsValid())
        return;

    m_EntityHandle.destruct();
}
