#pragma once

#include <flecs.h>
#include <glm/glm.hpp>

#include <string>
#include <vector>

/**
 * @brief Flecs entity handle을 GraspLink에서 쓰기 편한 API로 감싼 lightweight wrapper.
 *
 * @details
 * Entity 자체가 Component 데이터를 별도로 소유하는 것은 아니다. 내부에는 Flecs가 관리하는
 * `flecs::entity` handle만 보관하고 Transform/Hierarchy/Component 접근을 얇게 위임한다.
 *
 * 중요한 책임 분리는 다음과 같다.
 * - Entity: Local 값 읽기/쓰기와 Component 접근 인터페이스
 * - TransformSystemModule: Local TRS -> Local/World Matrix 계산
 * - Scene: Entity 생성과 Scene 단위 lifetime 관리
 *
 * 따라서 World Transform을 Entity가 직접 계산하거나 수정하지 않는다.
 *
 * @note Entity는 Flecs World를 소유하지 않는 non-owning handle이다. 원본 World가 먼저 파괴되면
 *       handle을 더 이상 사용하면 안 된다.
 * @todo [FUTURE] Transform Rotation 저장 방식을 quaternion으로 바꾸면 Local rotation API도 함께 정리한다.
 */
class Entity
{
public:
    /** @brief null Flecs handle을 가진 빈 Entity wrapper를 생성한다. */
    Entity() = default;

    /**
     * @brief 이미 존재하는 Flecs Entity handle을 wrapper로 감싼다.
     * @param handle Flecs World에 존재하는 entity handle.
     */
    explicit Entity(flecs::entity handle);

    Entity(const Entity&) = default;
    Entity& operator=(const Entity&) = default;

    /** @brief 부모 기준 Local position을 반환한다. Component가 없으면 (0,0,0)을 반환한다. */
    glm::vec3 GetLocalPosition() const;

    /** @brief 부모 기준 Local Euler rotation(radian)을 반환한다. */
    glm::vec3 GetLocalRotation() const;

    /** @brief 부모 기준 Local scale을 반환한다. Component가 없으면 (1,1,1)을 반환한다. */
    glm::vec3 GetLocalScale() const;

    /** @brief `(Position, Local)` Component를 갱신한다. */
    void SetLocalPosition(const glm::vec3& position);

    /**
     * @brief `(Rotation, Local)` Component를 Euler radian으로 갱신한다.
     * @note World Matrix는 즉시 직접 수정하지 않고 다음 TransformSystem 실행에서 재계산된다.
     */
    void SetLocalRotation(const glm::vec3& rotation);

    /** @brief `(Scale, Local)` Component를 갱신한다. */
    void SetLocalScale(const glm::vec3& scale);

    /**
     * @brief TransformSystem이 계산한 최종 World Matrix를 반환한다.
     * @return `(TransformMatrix, World)`가 없으면 identity matrix.
     */
    glm::mat4 GetWorldMatrix() const;

    /**
     * @brief 현재 Entity를 parent의 ChildOf 관계로 연결한다.
     * @param parent 새 부모 Entity.
     * @return chaining을 위한 자기 자신 reference.
     */
    Entity& SetParent(const Entity& parent);

    /**
     * @brief 전달된 child를 현재 Entity의 자식으로 연결한다.
     * @param child 연결할 자식 Entity.
     * @return chaining을 위한 자기 자신 reference.
     */
    Entity& AddChild(const Entity& child);

    /** @brief 직접 부모 Entity를 반환한다. 부모가 없으면 빈 Entity를 반환한다. */
    Entity GetParent() const;

    /** @brief 현재 Entity의 직속 자식들을 반환한다. */
    std::vector<Entity> GetChildren() const;

    /**
     * @brief 현재 Entity 바로 아래에서 이름이 일치하는 child를 찾는다.
     * @param name 찾을 Flecs Entity 이름.
     */
    Entity GetChild(const std::string& name) const;

    /**
     * @brief 하위 hierarchy 전체를 DFS(Depth First Search)로 탐색해 이름이 같은 Entity를 찾는다.
     * @param targetName 찾을 Entity 이름.
     * @return 찾은 Entity, 없으면 빈 Entity.
     *
     * @details HCR-12A의 `J1`~`J6`처럼 깊은 GLB hierarchy 내부의 논리 노드를 찾을 때 사용한다.
     */
    Entity FindChildByNameRecursive(const std::string& targetName) const;

    /**
     * @brief 일반 데이터 Component 값을 설정한다.
     * @tparam T Flecs에 등록 가능한 Component 타입.
     * @param component 설정할 값.
     */
    template<typename T>
    Entity& Set(const T& component)
    {
        m_EntityHandle.set<T>(component);
        return *this;
    }

    /**
     * @brief 값이 없는 Tag Component를 추가한다.
     * @tparam T 추가할 Tag 타입.
     */
    template<typename T>
    Entity& Add()
    {
        m_EntityHandle.add<T>();
        return *this;
    }

    /**
     * @brief 지정 Component/Tag를 제거한다.
     * @tparam T 제거할 Component 타입.
     */
    template<typename T>
    Entity& Remove()
    {
        m_EntityHandle.remove<T>();
        return *this;
    }

    /**
     * @brief 수정 가능한 Component reference를 얻는다.
     * @tparam T 읽고 수정할 Component 타입.
     * @warning 해당 Component가 존재한다는 전제에서 사용한다. 호출 전 Has<T>() 검사를 권장한다.
     */
    template<typename T>
    T& Get()
    {
        return m_EntityHandle.get_mut<T>();
    }

    /**
     * @brief 지정 Component 존재 여부를 반환한다.
     * @tparam T 확인할 Component 타입.
     */
    template<typename T>
    bool Has() const
    {
        return m_EntityHandle.has<T>();
    }

    /** @brief 내부 Flecs handle을 반환한다. */
    flecs::entity GetHandle() const
    {
        return m_EntityHandle;
    }

    /** @brief handle이 현재 World에서 살아 있는 Entity를 가리키는지 검사한다. */
    bool IsValid() const;

    /** @brief Flecs World에서 현재 Entity를 destruct한다. */
    void Destroy();

    /** @brief Flecs API에 wrapper를 직접 넘길 수 있도록 handle 변환을 제공한다. */
    operator flecs::entity() const
    {
        return m_EntityHandle;
    }

    /** @brief `if (entity)` 형태로 validity를 검사할 수 있게 한다. */
    explicit operator bool() const
    {
        return IsValid();
    }

    /** @brief 두 wrapper가 같은 Flecs Entity handle을 가리키는지 비교한다. */
    bool operator==(const Entity& other) const
    {
        return m_EntityHandle == other.m_EntityHandle;
    }

    /** @brief 두 wrapper의 Flecs handle이 다른지 비교한다. */
    bool operator!=(const Entity& other) const
    {
        return !(*this == other);
    }

private:
    flecs::entity m_EntityHandle{flecs::entity::null()};
};
