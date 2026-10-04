#pragma once

#include "Entity.h"

#include <flecs.h>

#include <string>

class SceneManager;

/**
 * @brief 하나의 장면에 속하는 Entity 생성과 Scene 단위 lifetime을 관리한다.
 *
 * @details
 * Scene은 Transform 계산/OpenGL draw/Robot control을 직접 수행하지 않는다.
 * - Scene: 어떤 Entity가 존재하는지와 Scene root lifetime 관리
 * - TransformSystemModule: Local/World Transform 계산
 * - RenderSystemModule: 렌더 가능한 Entity 선택
 * - Robotics Controller: RobotState 계산
 *
 * Scene마다 SceneRoot를 만들고 생성 Entity를 그 아래 ChildOf hierarchy로 묶는다.
 * Scene 전환 시 root hierarchy를 제거하면 해당 장면 Entity를 한 번에 정리할 수 있다.
 *
 * 시간/공간 규칙:
 * - OnUpdate(dt)의 dt 단위는 second [s].
 * - Entity Position/Node translation의 공간 단위는 로드한 asset/scene 규칙을 따른다.
 *   현재 controller-ready HCR scene은 meter [m] 기준이다.
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

    /** @brief 파생 Scene을 base pointer로 안전하게 삭제할 수 있도록 virtual destructor를 제공한다. */
    virtual ~Scene() = default;

    /** @brief Scene이 active가 된 직후, 첫 OnUpdate 전에 1회 호출된다. */
    virtual void OnEnter() {}

    /**
     * @brief active Scene에 대해 매 rendering frame 호출된다.
     * @param dt 이전 frame부터 경과한 시간 [s].
     * @note 고정 주기 robot/physics simulation은 장기적으로 이 variable frame dt와 분리하는 것이 목표다.
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
     * @return 생성된 Entity wrapper. 기본 local position=0, rotation=0 rad, scale=1.
     */
    Entity CreateEntity(const std::string& name = "");

    /** @return Scene이 사용하는 Flecs World reference. Scene이 소유하지 않는다. */
    flecs::world& GetWorld();

    /** @return 이 Scene hierarchy 전체 lifetime을 묶는 SceneRoot Flecs Entity handle. */
    flecs::entity GetSceneRoot() const;

private:
    friend class SceneManager;

    /** @brief Scene activation 전에 SceneRoot를 생성한다. SceneManager만 호출한다. */
    void InitRoot();

    /** @brief Scene 종료 시 SceneRoot와 하위 ChildOf hierarchy를 정리한다. SceneManager만 호출한다. */
    void CleanupRoot();

protected:
    /** @brief non-owning Flecs World reference. 실제 owner는 ViewerApp. */
    flecs::world& m_World;

    /** @brief Scene 단위 lifetime root. RobotRoot의 기구학/공간 root와는 별개다. */
    flecs::entity m_SceneRoot{flecs::entity::null()};
};
