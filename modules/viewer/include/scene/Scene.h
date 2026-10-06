#pragma once

#include "Entity.h"

#include <flecs.h>

#include <string>

class SceneManager;

/**
 * @brief 화면에 보이는 바닥과 로봇 같은 장면 물체를 만들고 한 묶음으로 정리하는 Scene이다.
 * @details
 * Entity는 장면 속 물체를 가리키고 Component는 위치·회전처럼 그 물체에 붙는 값이다. 실제 물체와 값은 Scene이 빌려 쓰는 Flecs World가 저장한다.
 * Scene은 자기 물체들을 SceneRoot라는 최상위 부모 아래 연결한다. root를 지우면 자식 관계로 묶인 바닥·로봇 등의 물체와 값도 함께 지워진다.
 * 변환 계산, 그리기, 물리 계산은 각각 별도 System이 맡는다. SceneRoot에는 위치·회전·크기를 주지 않고 자식들이 부모 변환을 누적할 기준으로 쓴다.
 *
 * SceneManager는 Scene 객체를 소유하지만 World는 외부에서 빌린다. Scene은 World보다 먼저 파괴되어야 한다.
 * `Entity::SetParent()`로 연결한 물체는 SceneRootTag로 정한 Scene 경계를 넘어 다른 Scene에 옮길 수 없다.
 */
class Scene
{
public:
    /**
     * @brief Scene 물체와 그 값을 저장할 Flecs World를 빌려 사용한다.
     * @param world Scene 객체와 SceneRoot 및 모든 장면 물체보다 오래 살아 있어야 하는 World 참조.
     */
    explicit Scene(flecs::world& world);

    virtual ~Scene() = default;

    /**
     * @brief Scene이 활성화될 때 SceneRoot 아래에 바닥이나 로봇 같은 초기 물체를 만든다.
     * @details SceneManager가 SceneRoot를 만든 다음 호출하므로 이곳에서 CreateEntity()를 사용할 수 있다.
     */
    virtual void OnEnter() {}

    /**
     * @brief 활성 Scene의 프레임별 물체 상태를 갱신한다.
     * @param dt 이전 호출 이후 흐른 시간 [s].
     */
    virtual void OnUpdate(float dt)
    {
        (void)dt;
    }

    /** @brief Scene을 바꾸거나 manager를 파괴하기 전에 Scene이 따로 보관한 자원을 정리한다. */
    virtual void OnExit() {}

    /**
     * @brief 활성 SceneRoot 아래에 위치·회전·크기를 저장할 장면 물체를 만든다.
     * @param name Flecs에서 이 Entity를 찾을 때 사용할 이름이다. 빈 문자열이면 이름을 붙이지 않는다.
     * @return 실제 물체를 소유하지 않고 그 물체를 가리키는 Entity 참조를 반환한다.
     * @throws std::logic_error Scene이 활성화되지 않았거나 root가 이미 정리된 경우.
     * @details 위치는 부모의 원점 `(0,0,0)`이고 회전은 부모와 같은 방향이며, 크기 배율은 각 축 1로 시작한다. TransformMatrix는 시스템이 이 값과 부모 변환을 합친 결과를 보관한다.
     * SceneRootTag는 어느 Scene이 물체를 정리하는지 표시한다. 예를 들어 바닥과 로봇을 같은 Scene root에 두면 Scene을 지울 때 함께 제거된다.
     */
    Entity CreateEntity(const std::string& name = "");

    /** @brief 장면 물체가 저장된 World를 참조로 돌려준다. 반환된 참조는 World 소유권을 넘기지 않는다. */
    flecs::world& GetWorld();

    /**
     * @brief 이 Scene의 물체들을 묶는 최상위 부모를 가리키는 handle을 반환한다.
     * @return Scene이 활성화되어 있으면 root handle을 반환하고, 아직 활성화되지 않았다면 null handle을 반환한다.
     * @details root는 각 장면 물체의 직접 또는 간접 부모다. Scene 전환으로 root가 삭제되면 그 handle도 사용할 수 없다.
     */
    flecs::entity GetSceneRoot() const;

private:
    friend class SceneManager;

    // SceneManager는 OnEnter()보다 먼저 root를 만들어 새 물체를 붙일 부모를 준비한다.
    void InitRoot();

    // OnExit()가 끝난 뒤 root를 지워 부모-자식으로 연결된 모든 장면 물체와 값을 World에서 제거한다.
    void CleanupRoot();

protected:
    // 이 World는 외부 소유다. Scene과 그 물체들을 정리할 수 있도록 World가 Scene보다 오래 살아야 한다.
    flecs::world& m_World;

    // SceneManager가 root를 만들고 지운다. root 삭제는 연결된 자손도 지우므로 외부에 남은 참조는 더는 유효하지 않다.
    flecs::entity m_SceneRoot{flecs::entity::null()};
};
