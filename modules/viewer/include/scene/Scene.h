#pragma once

#include "Entity.h"

#include <flecs.h>

#include <string>

class SceneManager;

// 한 Scene의 Entity hierarchy를 만들고 정리한다. Transform 계산과 렌더링은 각 System의 책임이다.
// SceneRoot는 삭제 경계다. Local TRS 없이 조상에서 유도된 World 행렬만 전달한다.
class Scene
{
public:
    // World는 외부 소유자에게서 빌린다. World가 Scene보다 오래 살아야 한다.
    explicit Scene(flecs::world& world);

    virtual ~Scene() = default;

    // 활성화 때 root 생성 후 한 번 호출한다. Entity 생성은 생성자 대신 여기서 시작한다.
    virtual void OnEnter() {}

    // 활성 Scene의 매 frame 갱신. dt 단위는 초.
    virtual void OnUpdate(float dt)
    {
        (void)dt;
    }

    // SceneRoot 정리 직전에 호출된다.
    virtual void OnExit() {}

    // 활성 root 아래에 기본 Local TRS와 행렬 저장 공간을 붙인다.
    // OnEnter 이전이나 Scene 정리 뒤에 호출하면 예외.
    Entity CreateEntity(const std::string& name = "");

    // 빌려 쓰는 Flecs World.
    flecs::world& GetWorld();

    // 이 Scene의 hierarchy root.
    flecs::entity GetSceneRoot() const;

private:
    friend class SceneManager;

    // SceneManager가 OnEnter 전에 호출한다.
    void InitRoot();

    // OnExit 다음 호출해 Scene hierarchy를 정리한다.
    void CleanupRoot();

protected:
    // ViewerApp 소유 World를 빌린다. Scene과 그 root 정리가 먼저 끝나야 한다.
    flecs::world& m_World;

    // root 삭제 시 ChildOf 자식도 함께 삭제된다. 반환한 Entity wrapper는 무효가 된다.
    flecs::entity m_SceneRoot{flecs::entity::null()};
};
