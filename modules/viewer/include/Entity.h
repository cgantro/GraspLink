#pragma once

#include <flecs.h>
#include <glm/glm.hpp>

#include <string>
#include <vector>

/**
 * @brief Flecs entity handle을 GraspLink에서 사용하기 쉬운 API로 감싼 lightweight non-owning wrapper.
 *
 * @details
 * Entity 자체가 Component 데이터를 별도로 소유하지 않는다. 내부에는 Flecs가 관리하는 `flecs::entity` handle만
 * 보관하고 Transform/Hierarchy/Component 접근을 얇게 위임한다.
 *
 * Transform 책임 분리:
 * - Entity: Local Position/Rotation/Scale 값을 읽고 쓴다.
 * - TransformSystemModule: Local TRS -> Local Matrix -> World Matrix를 계산한다.
 * - Scene: Entity 생성과 Scene 단위 lifetime을 관리한다.
 * - RobotTransformAdapter: RobotState [rad]를 Joint의 Local Rotation으로 변환한다.
 *
 * 현재 controller-ready HCR scene에서 Position/Matrix translation은 meter [m], Rotation은 Euler radian [rad]다.
 * Entity는 단위를 변환하지 않고 Component 값을 그대로 읽고 쓴다.
 *
 * @note Entity는 Flecs World를 소유하지 않는다. 원본 World가 먼저 파괴되면 handle을 사용하면 안 된다.
 * @todo [FUTURE] Transform Rotation 저장 방식을 quaternion으로 바꾸면 Local rotation API도 함께 정리한다.
 */
class Entity
{
public:
    /** @brief null Flecs handle을 가진 빈 Entity wrapper를 생성한다. */
    Entity() = default;

    /**
     * @brief 이미 존재하는 Flecs Entity handle을 wrapper로 감싼다.
     * @param handle Flecs World에 존재하는 entity handle. 소유권을 가져오지 않는다.
     */
    explicit Entity(flecs::entity handle);

    /** @brief 동일 Flecs handle을 가리키는 wrapper 복사를 허용한다. Component 자체가 복제되는 것은 아니다. */
    Entity(const Entity&) = default;

    /** @brief 다른 wrapper의 Flecs handle을 공유하도록 대입한다. */
    Entity& operator=(const Entity&) = default;

    /**
     * @brief 부모 기준 Local position을 반환한다.
     * @return `(Position,Local)` 값. Component가 없으면 (0,0,0). 현재 HCR scene에서는 [m].
     */
    glm::vec3 GetLocalPosition() const;

    /**
     * @brief 부모 기준 Local Euler rotation을 반환한다.
     * @return `(Rotation,Local)` XYZ Euler angle [rad]. Component가 없으면 구현의 기본값을 따른다.
     */
    glm::vec3 GetLocalRotation() const;

    /**
     * @brief 부모 기준 Local scale을 반환한다.
     * @return 무차원 xyz scale. Component가 없으면 (1,1,1).
     */
    glm::vec3 GetLocalScale() const;

    /**
     * @brief `(Position,Local)` Component를 갱신한다.
     * @param position 부모 frame 기준 translation. 현재 HCR scene에서는 [m].
     */
    void SetLocalPosition(const glm::vec3& position);

    /**
     * @brief `(Rotation,Local)` Component를 Euler radian으로 갱신한다.
     * @param rotation XYZ Euler angle [rad].
     * @note World Matrix는 즉시 직접 수정하지 않고 다음 TransformSystem 실행에서 재계산된다.
     */
    void SetLocalRotation(const glm::vec3& rotation);

    /**
     * @brief `(Scale,Local)` Component를 갱신한다.
     * @param scale 무차원 축별 배율. (1,1,1)이 원본 크기.
     */
    void SetLocalScale(const glm::vec3& scale);

    /**
     * @brief TransformSystem이 계산한 최종 World Matrix를 반환한다.
     * @return `(TransformMatrix,World)`가 없으면 identity matrix.
     * @note Matrix translation 성분의 공간 단위는 Scene/asset 단위를 따른다.
     */
    glm::mat4 GetWorldMatrix() const;

    /**
     * @brief 현재 Entity를 parent의 ChildOf 관계로 연결한다.
     * @param parent 새 부모 Entity.
     * @return chaining을 위한 자기 자신 reference.
     * @note Local transform 값은 자동으로 world-preserving 보정되지 않는다. 호출자가 hierarchy 의미를 알고 사용해야 한다.
     */
    Entity& SetParent(const Entity& parent);

    /**
     * @brief 전달된 child를 현재 Entity의 자식으로 연결한다.
     * @param child 연결할 자식 Entity.
     * @return chaining을 위한 자기 자신 reference.
     */
    Entity& AddChild(const Entity& child);

    /** @return 직접 부모 Entity. 부모가 없으면 빈 Entity wrapper. */
    Entity GetParent() const;

    /** @return 현재 Entity의 직속 자식 Entity wrapper 배열. */
    std::vector<Entity> GetChildren() const;

    /**
     * @brief 현재 Entity 바로 아래에서 이름이 일치하는 child를 찾는다.
     * @param name 찾을 Flecs Entity 이름.
     * @return 찾은 직접 자식, 없으면 빈 Entity.
     */
    Entity GetChild(const std::string& name) const;

    /**
     * @brief 하위 hierarchy 전체를 DFS(Depth First Search)로 탐색해 이름이 같은 Entity를 찾는다.
     * @param targetName 찾을 Entity 이름.
     * @return 찾은 Entity, 없으면 빈 Entity.
     *
     * @details HCR-12A의 J1~J6처럼 깊은 GLB hierarchy 내부의 논리 node binding에 사용한다.
     */
    Entity FindChildByNameRecursive(const std::string& targetName) const;

    /**
     * @brief 일반 데이터 Component 값을 설정한다.
     * @tparam T Flecs에 등록 가능한 Component 타입.
     * @param component 설정할 값.
     * @return chaining을 위한 자기 자신 reference.
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
     * @return chaining을 위한 자기 자신 reference.
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
     * @return chaining을 위한 자기 자신 reference.
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
     * @return Flecs가 관리하는 Component의 mutable reference.
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

    /** @return 내부 Flecs handle 값 복사본. Entity lifetime 소유권은 이전되지 않는다. */
    flecs::entity GetHandle() const
    {
        return m_EntityHandle;
    }

    /** @return handle이 현재 World에서 살아 있는 Entity를 가리키면 true. */
    bool IsValid() const;

    /** @brief Flecs World에서 현재 Entity와 Flecs가 정의한 관계/Component를 destruct한다. */
    void Destroy();

    /** @brief Flecs API에 wrapper를 직접 넘길 수 있도록 handle 값 변환을 제공한다. */
    operator flecs::entity() const
    {
        return m_EntityHandle;
    }

    /** @brief `if (entity)` 형태로 IsValid()를 검사하기 위한 명시적 bool 변환. */
    explicit operator bool() const
    {
        return IsValid();
    }

    /** @return 두 wrapper가 같은 Flecs Entity handle을 가리키면 true. */
    bool operator==(const Entity& other) const
    {
        return m_EntityHandle == other.m_EntityHandle;
    }

    /** @return 두 wrapper가 다른 Flecs Entity handle을 가리키면 true. */
    bool operator!=(const Entity& other) const
    {
        return !(*this == other);
    }

private:
    /** @brief Flecs World가 실제 lifetime을 소유하는 non-owning entity handle. */
    flecs::entity m_EntityHandle{flecs::entity::null()};
};
