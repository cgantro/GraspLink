#pragma once

#include "Entity.h"

#include <flecs.h>

#include <string>

class SceneManager;

/**
 * @brief 한 Flecs World 안에서 Entity 계층을 만들고 정리하는 Scene 기반 타입이다.
 * @details
 * Entity와 Component의 실제 저장소는 외부에서 빌린 Flecs World다. Scene은 별도 World를
 * 만들지 않고 SceneRoot를 계층 경계로 사용한다. root를 파괴하면 ChildOf 자손과 Component가
 * 제거되어 이전 Scene의 Entity handle이 무효가 된다. Transform, 렌더링, 물리 처리는 System 책임이다.
 * SceneRoot에는 Local TRS를 두지 않고 TransformSystem이 만든 World 행렬을 자손 기준으로 쓴다.
 * @note Scene은 빌린 World보다 먼저 파괴되어야 한다. SceneManager는 Scene을 소유하고 World를 빌린다.
 * `Entity::SetParent()`를 통하는 Scene 소속 Entity는 SceneRootTag 경계를 넘어 재부모화할 수 없다.
 */
class Scene
{
public:
    /** @brief 외부 소유 Flecs World를 빌린다. @param world Scene보다 오래 살아야 하는 World. */
    explicit Scene(flecs::world& world);

    virtual ~Scene() = default;

    /** @brief root가 준비된 뒤 활성화 때 호출된다. 여기서 CreateEntity()로 계층을 구성한다. */
    virtual void OnEnter() {}

    /** @brief 활성 Scene을 프레임마다 갱신한다. @param dt 경과 시간 [s]. */
    virtual void OnUpdate(float dt)
    {
        (void)dt;
    }

    /** @brief Scene 전환 또는 manager 파괴 때 root 정리 직전에 호출된다. */
    virtual void OnExit() {}

    /**
     * @brief 활성 root 아래에 기본 Local TRS와 행렬 저장 공간을 가진 Entity를 만든다.
     * @param name Flecs 이름. 빈 문자열이면 이름 없는 Entity를 만든다.
     * @return SceneRoot의 자식으로 연결된 비소유 Entity wrapper.
     * @throws std::logic_error root가 없거나 이미 정리된 경우.
     * @details 위치와 회전은 부모 기준 Local 원점, Scale은 단위 배율로 시작한다. TransformMatrix는
     * TransformSystem이 계산 결과를 저장할 자리다. 소속은 hierarchy의 SceneRootTag로 구분한다.
     * 일반 World Entity와 달리 이 API가 만드는 Entity는 공간 계층용 Local TRS를 갖고 root에 연결된다.
     */
    Entity CreateEntity(const std::string& name = "");

    /** @brief 빌린 Flecs World를 반환한다. 참조는 World 소유권을 이전하지 않는다. */
    flecs::world& GetWorld();

    /**
     * @brief Scene hierarchy의 root handle을 반환한다.
     * @return 활성 root 또는 아직 root가 없을 때 null handle.
     * @details 최상위 Scene Entity의 부모다. 반환 handle은 빌린 참조이며 Scene 전환 때 무효가 된다.
     */
    flecs::entity GetSceneRoot() const;

private:
    friend class SceneManager;

    // SceneManager가 OnEnter 직전에 root를 만든다.
    void InitRoot();

    // OnExit 뒤 root를 파괴해 ChildOf 자손 계층을 정리한다.
    void CleanupRoot();

protected:
    // 외부 World를 빌린다. Scene 및 root 정리가 World 파괴보다 먼저 끝나야 한다.
    flecs::world& m_World;

    // SceneManager만 생성·정리한다. destruct()는 ChildOf 자손도 파괴해 외부 handle을 무효화한다.
    flecs::entity m_SceneRoot{flecs::entity::null()};
};
