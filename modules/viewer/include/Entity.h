#pragma once

#include <flecs.h>
#include <glm/glm.hpp>

#include <string>
#include <vector>

/**
 * @brief Flecs Entity handle wrapper
 *
 * 보관: Entity handle만. Component 값은 Flecs World 소유
 * Local 값: 이 클래스에서 읽기·변경
 * World 행렬: TransformSystemModule 계산
 * 생성·삭제: Scene 담당
 * 수명: Flecs World보다 짧게 유지
 */
class Entity
{
public:
    /** @brief 빈 Entity handle */
    Entity() = default;

    /**
     * @brief 기존 Flecs Entity handle을 감쌈
     * @param handle Flecs World가 관리하는 Entity
     */
    explicit Entity(flecs::entity handle);

    Entity(const Entity&) = default;
    Entity& operator=(const Entity&) = default;

    /** @brief 부모 기준 위치 (Position이 없으면 0, 0, 0) */
    glm::vec3 GetLocalPosition() const;

    /** @brief 부모 기준 Euler 회전 (라디안) */
    glm::vec3 GetLocalRotation() const;

    /** @brief 부모 기준 크기 배율 (Scale이 없으면 1, 1, 1) */
    glm::vec3 GetLocalScale() const;

    /** @brief 부모 기준 위치 변경. 화면 행렬은 TransformSystemModule에서 갱신 */
    void SetLocalPosition(const glm::vec3& position);

    /**
     * @brief 부모 기준 Euler 회전 변경 (라디안)
     * @note 화면 행렬은 다음 TransformSystem 실행 때 갱신
     */
    void SetLocalRotation(const glm::vec3& rotation);

    /** @brief 부모 기준 크기 배율 변경 */
    void SetLocalScale(const glm::vec3& scale);

    /**
     * @brief 부모 변환을 반영한 World 행렬
     * @return 계산 전이면 기본 행렬
     */
    glm::mat4 GetWorldMatrix() const;

    /**
     * @brief 현재 Entity에 부모 설정
     * @param parent 새 부모 Entity
     * @return 호출 연결을 위한 자기 자신
     */
    Entity& SetParent(const Entity& parent);

    /**
     * @brief Entity를 자식으로 연결
     * @param child 연결할 자식 Entity
     * @return 호출 연결을 위한 자기 자신
     */
    Entity& AddChild(const Entity& child);

    /** @brief 부모 Entity (없으면 빈 handle) */
    Entity GetParent() const;

    /** @brief 직속 자식 Entity 목록 */
    std::vector<Entity> GetChildren() const;

    /**
     * @brief 직속 자식 중 이름으로 검색
     * @param name 검색할 Entity 이름
     */
    Entity GetChild(const std::string& name) const;

    /**
     * @brief 전체 하위 Entity에서 이름으로 검색
     * @param targetName 검색할 Entity 이름
     * @return 일치 항목 (없으면 빈 handle)
     *
     * 용도: GLB 계층 안쪽의 J1~J6 관절 Entity 검색
     */
    Entity FindChildByNameRecursive(const std::string& targetName) const;

    /**
     * @brief Component 추가 또는 값 변경
     * @tparam T Component 타입
     * @param component 저장할 Component 값
     */
    template<typename T>
    Entity& set(const T& component)
    {
        m_EntityHandle.set<T>(component);
        return *this;
    }

    /**
     * @brief 값 없는 Tag 추가
     * @tparam T Tag 타입
     */
    template<typename T>
    Entity& Add()
    {
        m_EntityHandle.add<T>();
        return *this;
    }

    /**
     * @brief Component 또는 Tag 제거
     * @tparam T 제거할 타입
     */
    template<typename T>
    Entity& Remove()
    {
        m_EntityHandle.remove<T>();
        return *this;
    }

    /**
     * @brief 수정 가능한 Component 값
     * @tparam T 가져올 Component 타입
     * @warning Component 존재 확인 후 호출 (Has<T>())
     */
    template<typename T>
    T& Get()
    {
        return m_EntityHandle.get_mut<T>();
    }

    /**
     * @brief Component 존재 여부
     * @tparam T 확인할 타입
     */
    template<typename T>
    bool Has() const
    {
        return m_EntityHandle.has<T>();
    }

    /** @brief 내부 Flecs Entity handle */
    flecs::entity GetHandle() const
    {
        return m_EntityHandle;
    }

    /** @brief Entity handle 유효성 */
    bool IsValid() const;

    /** @brief Flecs World에서 Entity 삭제 */
    void Destroy();

    /** @brief Flecs 함수에 내부 Entity 전달 */
    operator flecs::entity() const
    {
        return m_EntityHandle;
    }

    /** @brief `if (entity)` 형태의 유효성 확인 */
    explicit operator bool() const
    {
        return IsValid();
    }

    /** @brief 두 handle이 같은 Entity인지 비교 */
    bool operator==(const Entity& other) const
    {
        return m_EntityHandle == other.m_EntityHandle;
    }

    /** @brief 두 handle이 다른 Entity인지 비교 */
    bool operator!=(const Entity& other) const
    {
        return !(*this == other);
    }

private:
    flecs::entity m_EntityHandle{flecs::entity::null()};
};
