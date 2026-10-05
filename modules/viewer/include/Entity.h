#pragma once

#include <flecs.h>
#include <glm/glm.hpp>

#include <string>
#include <vector>

// Entity와 Component는 Flecs World가 소유한다. wrapper를 복사해도 수명은 늘어나지 않는다.
// World가 살아 있는 동안만 사용하며, Entity 삭제나 Scene 정리 뒤에는 handle이 무효가 된다.
class Entity
{
public:
    Entity() = default;

    explicit Entity(flecs::entity handle);

    Entity(const Entity&) = default;
    Entity& operator=(const Entity&) = default;

    // 좌표: 부모 기준 위치 [m], Euler 회전 [rad], 크기는 단위 없는 배율.
    // 해당 Local Component가 없거나 handle이 무효면 위치·회전은 0, 크기는 1.
    glm::vec3 GetLocalPosition() const;

    glm::vec3 GetLocalRotation() const;

    glm::vec3 GetLocalScale() const;

    // Local 값만 변경한다. World 행렬은 다음 TransformSystemModule 갱신까지 이전 값이다.
    void SetLocalPosition(const glm::vec3& position);

    void SetLocalRotation(const glm::vec3& rotation);

    void SetLocalScale(const glm::vec3& scale);

    // TransformSystemModule이 마지막으로 계산해 둔 Scene 기준 행렬을 반환한다.
    glm::mat4 GetWorldMatrix() const;

    // 같은 World 안에서 부모 변경. Scene 소속 Entity는 같은 Scene 안에서만 이동 가능.
    // World/Scene 경계를 넘거나 자기 자신·자손 아래로 이동하면 예외.
    // Local TRS는 유지되어 World 위치가 달라질 수 있다.
    Entity& SetParent(const Entity& parent);

    Entity& AddChild(const Entity& child);

    // 부모가 없으면 빈 handle.
    Entity GetParent() const;

    std::vector<Entity> GetChildren() const;

    // 이름으로 자식 검색.
    Entity GetChild(const std::string& name) const;

    // 하위 hierarchy 전체에서 이름 검색. 없으면 빈 handle.
    Entity FindChildByNameRecursive(const std::string& targetName) const;

    template<typename T>
    Entity& set(const T& component)
    {
        m_EntityHandle.set<T>(component);
        return *this;
    }

    template<typename T>
    Entity& Add()
    {
        m_EntityHandle.add<T>();
        return *this;
    }

    template<typename T>
    Entity& Remove()
    {
        m_EntityHandle.remove<T>();
        return *this;
    }

    // 먼저 Has<T>()로 존재를 확인한다. Flecs 구조 변경을 넘겨 이 참조를 보관하지 않는다.
    template<typename T>
    T& Get()
    {
        return m_EntityHandle.get_mut<T>();
    }

    template<typename T>
    bool Has() const
    {
        return m_EntityHandle.has<T>();
    }

    flecs::entity GetHandle() const
    {
        return m_EntityHandle;
    }

    bool IsValid() const;

    // ChildOf 자식까지 삭제한다. 같은 Entity를 가리키던 다른 wrapper도 무효가 된다.
    void Destroy();

    operator flecs::entity() const
    {
        return m_EntityHandle;
    }

    explicit operator bool() const
    {
        return IsValid();
    }

    bool operator==(const Entity& other) const
    {
        return m_EntityHandle == other.m_EntityHandle;
    }

    bool operator!=(const Entity& other) const
    {
        return !(*this == other);
    }

private:
    flecs::entity m_EntityHandle{flecs::entity::null()};
};
