#pragma once

#include <flecs.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <string>
#include <vector>

namespace grasplink::scene
{

/**
 * @brief 장면 속 한 물체의 ID를 보관하고 그 물체의 값을 다루는 Entity 참조다.
 * @details
 * Entity는 Viewer 장면 안의 물체 하나를 뜻한다. Component는 그 물체에 붙는 위치, 회전, 크기 같은 값이다.
 * 이 클래스는 물체와 Component를 직접 담지 않고 Flecs가 발급한 handle만 보관하므로 복사해도 대상 물체의 수명이 늘어나지 않는다.
 * Flecs World가 물체와 Component를 소유하고, Scene은 자기 root 아래에 연결된 물체들을 정리한다.
 * 따라서 Scene은 World보다 먼저 끝나야 하며, World가 끝난 뒤에는 이 handle을 사용하면 안 된다.
 */
class Entity
{
public:
    /** @brief 아직 장면 속 물체를 가리키지 않는 빈 Entity 참조를 만든다. */
    Entity() = default;

    /** @brief 기존 장면 물체의 handle을 보관해 접근 메서드를 제공한다. 물체의 소유권은 가져오지 않는다. */
    explicit Entity(flecs::entity handle);

    Entity(const Entity&) = default;
    Entity& operator=(const Entity&) = default;

    /**
     * @brief 바로 위 부모를 기준으로 저장된 Local 위치를 반환한다.
     * @details Local은 부모 기준 값이고 World는 Scene 전체 기준 값이다. 반환 단위는 [m]이며, 예를 들어 로봇 링크의 Local 위치는 그 링크의 부모 관절 기준이다.
     * 위치 값이 없거나 물체가 이미 삭제되었다면 원점 `(0, 0, 0)`을 반환한다.
     */
    glm::vec3 GetLocalPosition() const;

    /**
     * @brief 바로 위 부모를 기준으로 저장된 Local 회전을 반환한다.
     * @details 회전은 네 수로 방향을 표현하는 quaternion으로 저장하며 GLM 성분 순서는 `(w,x,y,z)`다.
     * 값이 없거나 물체가 삭제되었다면 방향을 바꾸지 않는 `(1,0,0,0)`을 반환한다.
     */
    glm::quat GetLocalRotation() const;

    /**
     * @brief 바로 위 부모 기준으로 저장된 축별 크기 배율을 반환한다.
     * @details 배율에는 단위가 없다. 값이 없거나 물체가 삭제되었다면 크기를 바꾸지 않는 `(1,1,1)`을 반환한다.
     */
    glm::vec3 GetLocalScale() const;

    /**
     * @brief 바로 위 부모를 기준으로 물체의 위치를 설정한다.
     * @param position 부모 원점에서 물체 원점까지의 위치 [m]다.
     * @details Local 위치만 바뀌고 Scene 전체 위치를 나타내는 World 행렬은 다음 TransformSystemModule 호출에서 갱신된다. 물체가 삭제된 뒤 호출하면 아무 작업도 하지 않는다.
     */
    void SetLocalPosition(const glm::vec3& position);

    /**
     * @brief 부모 기준 회전을 저장하기 전에 quaternion 길이를 1로 맞춘다.
     * @param rotation 부모 기준 방향을 나타내는 네 수 (w,x,y,z)다. 각 성분은 유한해야 한다.
     * @throws std::invalid_argument 대상이 살아 있고 rotation이 모두 0이거나 유한하지 않은 성분을 가진 경우.
     * @details 잘못된 입력이면 기존 방향을 유지하고 예외를 던진다. World 행렬은 다음 TransformSystemModule 호출에서 조상 방향까지 합쳐 갱신한다. 삭제된 대상에는 아무 작업도 하지 않는다.
     */
    void SetLocalRotation(const glm::quat& rotation);

    /**
     * @brief 바로 위 부모 기준으로 물체의 축별 크기 배율을 설정한다.
     * @param scale 각 축의 크기를 몇 배로 바꿀지 나타낸다. `(1,1,1)`이면 크기가 바뀌지 않는다.
     * @details 새 배율은 Local 값으로 저장된다. Scene 전체 크기를 반영한 행렬은 다음 TransformSystemModule 호출 때 계산된다.
     */
    void SetLocalScale(const glm::vec3& scale);

    /**
     * @brief 부모 변환까지 합쳐 마지막으로 계산한 Scene 기준 행렬을 반환한다.
     * @return 마지막으로 계산한 Scene 기준 행렬이다. Entity가 파괴되었거나 World 행렬이 아직 없으면 항등 행렬을 반환한다.
     * @details World는 Scene 전체 기준이라는 뜻으로 Local의 부모 기준 값과 구별된다. 위치·회전·크기 또는 부모를 바꿔도 행렬은 즉시 바뀌지 않는다.
     * 렌더링이나 물리가 새 자세를 읽기 전에 TransformSystemModule이 행렬을 다시 계산해야 한다.
     */
    glm::mat4 GetWorldMatrix() const;

    /**
     * @brief 이 장면 물체를 다른 물체 아래에 연결해 부모와 자식 관계를 바꾼다.
     * @param parent 이 Entity의 새 부모로 연결할 대상이다.
     * @return 메서드 호출을 이어갈 수 있도록 이 wrapper의 참조를 반환한다.
     * @throws std::invalid_argument 두 대상이 다른 Flecs World에 속하거나 Scene 경계를 넘어 이동시키거나 부모-자식 순환을 만들 때.
     * @details 두 대상은 같은 Flecs World와 Scene에 속해야 하며 순환 부모 관계는 허용하지 않는다. 현재 대상이나 새 부모가 무효하면 바꾸지 않는다.
     * 부모가 바뀌어도 위치·회전·크기는 새 부모 기준으로 다시 해석되므로 Scene에서 보이는 자세가 달라질 수 있다.
     * 화면의 자세를 유지하려면 호출자가 새 부모 기준 값을 다시 지정하고 TransformSystemModule을 실행해야 한다.
     */
    Entity& SetParent(const Entity& parent);

    /**
     * @brief 지정한 장면 물체를 이 물체 아래에 연결한다.
     * @param child 이 Entity 아래에 연결할 자식 대상이다.
     * @return 메서드 호출을 이어갈 수 있도록 이 wrapper의 참조를 반환한다.
     * @details SetParent()가 같은 저장소와 Scene인지, 순환 관계가 생기는지 확인한다. 어느 한쪽이 삭제된 상태면 관계를 바꾸지 않는다.
     */
    Entity& AddChild(const Entity& child);

    /** @brief 바로 위 부모 물체를 가리키는 Entity 참조를 반환한다. 반환값은 물체를 소유하지 않는다. */
    Entity GetParent() const;

    /** @brief 바로 아래 자식 물체들을 가리키는 Entity 참조 목록을 반환한다. 목록은 물체를 소유하지 않는다. */
    std::vector<Entity> GetChildren() const;

    /**
     * @brief 이름 또는 경로로 이 물체 바로 아래에 연결된 자식을 찾는다.
     * @param name 자식 물체의 이름 또는 Flecs가 찾을 경로다.
     * @return 대상을 찾으면 그 Entity를 가리키는 wrapper를, 찾지 못하면 빈 wrapper를 반환한다.
     */
    Entity GetChild(const std::string& name) const;

    /**
     * @brief 자식부터 아래로 따라가며 이름이 같은 장면 물체를 찾는다.
     * @param targetName 자손 Entity 가운데 찾을 이름이다.
     * @return 가장 먼저 찾은 Entity를 가리키는 wrapper를 반환하며, 없으면 빈 wrapper를 반환한다.
     * @details 이름 없는 중간 물체 아래도 계속 탐색한다. 검색을 시작한 현재 물체 자신은 검사하지 않는다.
     */
    Entity FindChildByNameRecursive(const std::string& targetName) const;

    /**
     * @brief 위치나 회전처럼 이 물체에 붙는 값을 새로 저장하거나 기존 값을 바꾼다.
     * @tparam T 저장할 값의 C++ 타입이다.
     * @param component 물체에 붙여 저장할 값이다.
     * @return 다른 Entity 메서드를 이어서 호출할 수 있도록 이 wrapper를 반환한다.
     * @details 위치나 회전 같은 값은 이 Entity 참조가 아니라 Flecs World에 저장된다. World와 대상 물체가 살아 있는 동안 호출해야 한다.
     */
    template<typename T>
    Entity& set(const T& component)
    {
        m_EntityHandle.set<T>(component);
        return *this;
    }

    /**
     * @brief 지정한 종류의 값을 기본값으로 이 물체에 추가한다.
     * @tparam T 추가할 값의 C++ 타입이다.
     * @return 다른 Entity 메서드를 이어서 호출할 수 있도록 이 wrapper를 반환한다.
     */
    template<typename T>
    Entity& Add()
    {
        m_EntityHandle.add<T>();
        return *this;
    }

    /**
     * @brief 이 물체에서 지정한 종류의 값을 제거한다.
     * @tparam T 제거할 값의 C++ 타입이다.
     * @return 다른 Entity 메서드를 이어서 호출할 수 있도록 이 wrapper를 반환한다.
     */
    template<typename T>
    Entity& Remove()
    {
        m_EntityHandle.remove<T>();
        return *this;
    }

    /**
     * @brief 이 물체에 이미 저장된 값을 수정할 수 있도록 참조를 가져온다.
     * @tparam T 가져올 값의 C++ 타입이다.
     * @return Flecs World 저장소에 있는 값의 수정 가능한 참조다.
     * @details 먼저 Has<T>()로 값이 있는지 확인해야 한다. 반환 참조는 World의 저장 공간을 가리키므로 다른 값을 추가하거나 제거한 뒤에는 보관해 두고 쓰지 않는다.
     */
    template<typename T>
    T& Get()
    {
        return m_EntityHandle.get_mut<T>();
    }

    /** @brief 지정한 종류의 값이 이 물체에 저장되어 있는지 확인한다. */
    template<typename T>
    bool Has() const
    {
        return m_EntityHandle.has<T>();
    }

    /** @brief 대상 물체를 식별하는 Flecs handle 복사본을 반환한다. 물체의 소유권은 이전되지 않는다. */
    flecs::entity GetHandle() const
    {
        return m_EntityHandle;
    }

    /** @brief Flecs World에 대상 물체가 아직 남아 있는지 확인한다. */
    bool IsValid() const;

    /**
     * @brief Flecs World에서 이 장면 물체를 삭제한다.
     * @details 부모-자식 관계로 연결된 모든 자손도 함께 제거된다. 같은 물체를 가리키는 다른 Entity 참조가 남아 있어도 모두 더는 사용할 수 없다. 이미 삭제되었다면 아무 작업도 하지 않는다.
     */
    void Destroy();

    /** @brief Entity handle을 직접 받는 Flecs API에 보관한 식별자를 전달한다. */
    operator flecs::entity() const
    {
        return m_EntityHandle;
    }

    /** @brief 조건식에서 이 참조가 살아 있는 장면 물체를 가리키는지 확인한다. */
    explicit operator bool() const
    {
        return IsValid();
    }

    /** @brief 두 참조가 같은 장면 물체를 가리키는지 비교한다. */
    bool operator==(const Entity& other) const
    {
        return m_EntityHandle == other.m_EntityHandle;
    }

    /** @brief 두 참조가 서로 다른 장면 물체를 가리키는지 비교한다. */
    bool operator!=(const Entity& other) const
    {
        return !(*this == other);
    }

private:
    flecs::entity m_EntityHandle{flecs::entity::null()};
};

}
