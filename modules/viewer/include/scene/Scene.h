#pragma once

#include "Entity.h"

#include <flecs.h>

#include <string>

class SceneManager;

/**
 * @brief 하나의 장면에 속하는 Entity 생성과 Scene 단위 lifetime을 관리한다.
 *
 * @details
 * Scene은 Transform 계산이나 OpenGL 렌더링을 직접 하지 않는다.
 * - Scene: 어떤 Entity가 존재하는지 관리
 * - TransformSystemModule: Local/World Transform 계산
 * - RenderSystemModule: 그릴 Entity 선택
 *
 * Scene마다 SceneRoot를 만들고 생성되는 Entity를 그 아래에 묶는다.
 * 따라서 Scene 전환 시 root hierarchy를 제거해 장면 전체를 정리할 수 있다.
 *
 * @todo [FUTURE] Robot/Target/Physics 환경을 생성하는 SimulationScene을 별도 파생 클래스로 추가한다.
 */
class Scene
{
public:
    /**
     * @brief Scene이 사용할 Flecs World를 연결한다.
     * @param world ViewerApp이 소유하는 ECS World.
     * @note Scene은 World를 소유하지 않으므로 World가 Scene보다 오래 살아야 한다.
     */
    explicit Scene(flecs::world& world);

    virtual ~Scene() = default;

    /** @brief Scene이 active가 된 직후 1회 호출된다. */
    virtual void OnEnter() {}

    /**
     * @brief active Scene에 대해 매 frame 호출된다.
     * @param dt 이전 frame부터 경과한 시간(second).
     */
    virtual void OnUpdate(float dt)
    {
        (void)dt;
    }

    /** @brief Scene이 교체/종료되기 직전에 1회 호출된다. */
    virtual void OnExit() {}

    /**
     * @brief 기본 Local/World Transform Component가 붙은 Scene Entity를 생성한다.
     * @param name Flecs Entity 이름. 빈 문자열도 허용한다.
     * @return 생성된 Entity wrapper.
     */
    Entity CreateEntity(const std::string& name = "");

    /** @brief Scene이 사용하는 Flecs World reference를 반환한다. */
    flecs::world& GetWorld();

    /** @brief 이 Scene hierarchy의 lifetime root Entity를 반환한다. */
    flecs::entity GetSceneRoot() const;

private:
    friend class SceneManager;

    /** @brief Scene activation 전에 SceneRoot를 생성한다. */
    void InitRoot();

    /** @brief Scene 종료 시 SceneRoot와 하위 hierarchy를 정리한다. */
    void CleanupRoot();

protected:
    /// non-owning reference. 실제 owner는 ViewerApp의 m_World다.
    flecs::world& m_World;

    /// Scene 단위 lifetime 관리용 root. RobotRoot의 공간 transform 역할과는 별개다.
    flecs::entity m_SceneRoot{flecs::entity::null()};
};
