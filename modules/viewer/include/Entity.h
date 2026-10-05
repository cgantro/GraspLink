#pragma once

#include <flecs.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <string>
#include <vector>

/**
 * @brief Flecs Entity를 가리키는 가벼운 wrapper다.
 * @details
 * Entity와 Component의 실제 소유자는 Flecs World다. 이 wrapper를 복사해도 Entity의 수명은
 * 늘어나지 않으며, Scene이 World에서 root hierarchy를 정리하거나 Destroy()를 호출하면
 * 같은 Entity를 가리키는 모든 wrapper가 무효가 된다. World 자체가 먼저 파괴된 뒤에는
 * wrapper를 사용하지 않는다. Scene을 쓰는 경우 Scene과 그 정리를 관리하는 SceneManager가
 * World보다 먼저 끝나야 한다.
 */
class Entity
{
public:
    /** @brief 무효한 Entity wrapper를 만든다. */
    Entity() = default;

    /** @brief 기존 Flecs handle을 감싼다. handle이나 Entity의 소유권은 가져오지 않는다. */
    explicit Entity(flecs::entity handle);

    Entity(const Entity&) = default;
    Entity& operator=(const Entity&) = default;

    /**
     * @brief 부모 기준 Local 위치를 반환한다.
     * @details 단위는 [m]. Position, Local pair가 없거나 handle이 무효하면 (0, 0, 0).
     */
    glm::vec3 GetLocalPosition() const;

    /**
     * @brief 부모 기준 Local quaternion 회전을 반환한다.
     * @details GLM 생성자 순서는 (w,x,y,z), 성분은 무차원. Rotation, Local pair가
     * 없거나 handle이 무효하면 항등 (1,0,0,0).
     */
    glm::quat GetLocalRotation() const;

    /**
     * @brief 부모 기준 Local 축별 크기 배율을 반환한다.
     * @details 단위 없는 배율이며 Scale, Local pair가 없거나 handle이 무효하면 (1, 1, 1).
     */
    glm::vec3 GetLocalScale() const;

    /**
     * @brief 부모 기준 위치를 설정한다.
     * @param position Local 위치 [m].
     * @details Local Position만 바꾼다. World 행렬은 TransformSystemModule이 다시 갱신될
     * 때까지 이전 캐시를 유지하며, 무효 handle이면 아무 작업도 하지 않는다.
     */
    void SetLocalPosition(const glm::vec3& position);

    /**
     * @brief 부모 기준 quaternion을 정규화해 Local 회전을 설정한다.
     * @param rotation 유한한 영벡터 아닌 quaternion. GLM 생성자 순서는 (w,x,y,z).
     * @throws std::invalid_argument 유효한 Entity에 영 quaternion 또는 비유한 성분을 전달할 때.
     * @details 검증 실패 시 이전 Local 회전을 유지한다. 무효 handle에는 아무 작업도 하지 않는다.
     * World 행렬은 다음 transform 갱신 전까지 이전 캐시다. 부모 자세와의 합성은 TransformSystem이 맡는다.
     */
    void SetLocalRotation(const glm::quat& rotation);

    /**
     * @brief 부모 기준 축별 크기 배율을 설정한다.
     * @param scale 단위 없는 Local 배율. (1, 1, 1)은 크기 변경이 없는 상태다.
     * @details Local Scale만 바꾸며 World 행렬은 다음 transform 갱신까지 다시 계산되지 않는다.
     */
    void SetLocalScale(const glm::vec3& scale);

    /**
     * @brief 마지막 transform 갱신에서 계산한 Scene 기준 행렬을 반환한다.
     * @return 캐시된 World 행렬. handle이 무효이거나 World 행렬 pair가 없으면 항등 행렬.
     * @details Local setter나 부모 변경 직후 자동 재계산하지 않는다. 최신 결과가 필요하면
     * 렌더링 또는 물리가 읽기 전에 TransformSystemModule을 갱신해야 한다.
     */
    glm::mat4 GetWorldMatrix() const;

    /**
     * @brief 이 Entity의 부모를 바꾸고 Flecs hierarchy에 연결한다.
     * @param parent 새 부모 Entity.
     * @return 이 wrapper에 대한 참조.
     * @throws std::invalid_argument 두 handle이 서로 다른 Flecs World에 속하거나,
     * Scene 소속 Entity를 다른 Scene 경계 밖으로 옮기거나, 계층 순환을 만들 때.
     * @details 무효한 이 Entity 또는 부모가 전달되면 변경 없이 반환한다. Scene에 속한
     * Entity는 같은 Scene 안에서만 재부모화할 수 있다. 위치·회전·크기의 Local 값은
     * 그대로 유지되므로 부모가 바뀌면 World 공간의 자세와 위치도 달라진다. World 자세를
     * 보존하도록 Local 값을 역산하지 않으며, 갱신된 World 행렬은 다음 transform 갱신 때 계산된다.
     */
    Entity& SetParent(const Entity& parent);

    /**
     * @brief child를 이 Entity의 자식으로 연결한다.
     * @param child 자식으로 둘 Entity.
     * @return 이 wrapper에 대한 참조.
     * @details 실제 검증과 재부모화는 SetParent()에 맡긴다. 둘 중 하나가 무효하면 변경하지 않는다.
     */
    Entity& AddChild(const Entity& child);

    /** @brief Flecs가 반환하는 즉시 부모 handle을 wrapper로 돌려준다. */
    Entity GetParent() const;

    /** @brief 즉시 자식들을 Flecs 순회 순서대로 wrapper 목록으로 반환한다. */
    std::vector<Entity> GetChildren() const;

    /**
     * @brief Flecs 이름 조회로 자식 hierarchy 안의 이름을 찾는다.
     * @param name 찾을 이름.
     * @return 찾은 handle 또는 찾지 못했을 때 무효 wrapper.
     */
    Entity GetChild(const std::string& name) const;

    /**
     * @brief 하위 hierarchy를 깊이 우선으로 순회해 이름이 같은 Entity를 찾는다.
     * @param targetName 찾을 이름.
     * @return 첫 번째 일치 Entity 또는 없을 때 무효 wrapper.
     * @details 이름 없는 중간 Entity도 계속 내려가므로 익명 grouping node 아래의 이름 있는
     * Entity도 검색된다. 현재 Entity 자신은 후보가 아니며 자손만 검색한다.
     */
    Entity FindChildByNameRecursive(const std::string& targetName) const;

    /**
     * @brief Component 값을 Flecs Entity에 설정하거나 갱신한다.
     * @tparam T Component 타입.
     * @param component 저장할 값.
     * @return 이 wrapper에 대한 참조.
     * @details Component 수명은 World가 관리한다. World/handle이 유효한 동안만 호출한다.
     */
    template<typename T>
    Entity& set(const T& component)
    {
        m_EntityHandle.set<T>(component);
        return *this;
    }

    /**
     * @brief 지정한 Component를 Entity에 추가한다.
     * @tparam T Component 타입.
     * @return 이 wrapper에 대한 참조.
     */
    template<typename T>
    Entity& Add()
    {
        m_EntityHandle.add<T>();
        return *this;
    }

    /**
     * @brief 지정한 Component를 Entity에서 제거한다.
     * @tparam T Component 타입.
     * @return 이 wrapper에 대한 참조.
     */
    template<typename T>
    Entity& Remove()
    {
        m_EntityHandle.remove<T>();
        return *this;
    }

    /**
     * @brief 현재 Entity의 Component를 수정 가능한 참조로 가져온다.
     * @tparam T Component 타입.
     * @return World가 소유하는 Component에 대한 참조.
     * @details 호출 전에 Has<T>()로 존재를 확인한다. 참조는 World 내부 저장소를 가리키므로
     * Component 추가·제거 등 구조 변경을 넘겨 보관하지 않는다.
     */
    template<typename T>
    T& Get()
    {
        return m_EntityHandle.get_mut<T>();
    }

    /** @brief 지정한 Component가 현재 Entity에 있는지 확인한다. */
    template<typename T>
    bool Has() const
    {
        return m_EntityHandle.has<T>();
    }

    /** @brief 감싼 Flecs handle을 반환한다. 소유권은 이전되지 않는다. */
    flecs::entity GetHandle() const
    {
        return m_EntityHandle;
    }

    /** @brief Flecs가 이 handle을 현재 World에서 살아 있는 Entity로 인식하는지 확인한다. */
    bool IsValid() const;

    /**
     * @brief 이 Entity를 Flecs에서 파괴한다.
     * @details ChildOf hierarchy의 자식도 함께 정리된다. 이 Entity를 가리키던 모든 wrapper는
     * 이후 무효가 된다. 이미 무효한 handle이면 아무 작업도 하지 않는다.
     */
    void Destroy();

    /** @brief 암시적으로 Flecs API에 전달할 때 감싼 handle을 제공한다. */
    operator flecs::entity() const
    {
        return m_EntityHandle;
    }

    /** @brief handle의 유효 여부를 bool 문맥에서 확인한다. */
    explicit operator bool() const
    {
        return IsValid();
    }

    /** @brief Flecs handle 동일성을 비교한다. */
    bool operator==(const Entity& other) const
    {
        return m_EntityHandle == other.m_EntityHandle;
    }

    /** @brief Flecs handle 비동일성을 비교한다. */
    bool operator!=(const Entity& other) const
    {
        return !(*this == other);
    }

private:
    flecs::entity m_EntityHandle{flecs::entity::null()};
};
