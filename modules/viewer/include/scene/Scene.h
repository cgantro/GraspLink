#pragma once

#include "Entity.h"

#include <flecs.h>

#include <string>

class SceneManager;

/*
    ============================================================================
    Scene
    ============================================================================

    하나의 "장면"에 속하는 Entity들의 생성과 생명주기를 관리한다.
    Scene이 직접 렌더링하거나 Transform을 계산하지는 않는다.
        Scene
            = 어떤 Entity들이 이 장면에 존재하는가?
        TransformSystemModule
            = Entity의 Local / World Transform 계산
        RenderSystemModule
            = Render 가능한 Entity를 찾아 Renderer에 전달


    Scene의 가장 중요한 역할 중 하나는 SceneRoot를 만드는 것이다.
        SceneRoot
        ├─ Cube
        ├─ Robot
        │   └─ ...
        └─ Object


    모든 Scene Entity를 Flecs ChildOf 관계로 SceneRoot 아래에 묶는다.

    따라서 Scene 전환 시 SceneRoot를 제거하면
    해당 Scene에 속하는 Entity hierarchy도 함께 정리할 수 있다.

    이후 로봇 Scene은 대략:
        SceneRoot
        ├─ RobotRoot
        │   └─ Base
        │       └─ J1
        │           └─ Link1
        │               └─ ...
        └─ TargetObject

    형태가 된다.
*/
class Scene
{
public:
    /*
        Scene은 Flecs World를 소유하지 않는다.

        ViewerApp / Engine 계층이 World를 소유하고,
        Scene은 reference만 보관한다.

        따라서 Scene보다 flecs::world가 오래 살아 있어야 한다.
    */
    explicit Scene(flecs::world& world);
    virtual ~Scene() = default;
    /*
        =========================================================================
        Scene Lifecycle
        =========================================================================

        SceneManager가 호출한다.

        OnEnter()
            Scene이 활성화될 때 1회 호출.

            이후:
                Robot 생성
                Object 생성
                Environment 생성

            등을 넣게 된다.


        OnUpdate()
            활성 Scene에 대해 매 frame 호출.


        OnExit()
            Scene이 교체되거나 종료되기 전에 호출.
    */
    virtual void OnEnter() {}

    virtual void OnUpdate(float dt)
    {
        (void)dt;
    }

    virtual void OnExit() {}


    /*
        =========================================================================
        CreateEntity
        =========================================================================

        Scene 내부의 "공간 Entity"를 생성하는 기본 진입점.

        직접

            m_World.entity(...)

        를 계속 호출하지 않고 Scene을 통해 생성하는 이유는
        모든 Scene Entity가 동일한 기본 상태를 갖도록 하기 위해서다.

        생성되는 Entity에는 자동으로:

            (Position, Local)
            (Rotation, Local)
            (Scale, Local)

            (TransformMatrix, Local)
            (TransformMatrix, World)

        가 붙는다.

        따라서 호출하는 쪽에서는:

            Entity cube = CreateEntity("Cube");

            cube.SetLocalPosition(...);

        정도만 작성하면 된다.
    */
    Entity CreateEntity(const std::string& name = "");

    /*
        Scene이 사용하는 World 접근.

        Scene 하위 구현에서 query 또는 Entity 생성 등이
        필요할 때 사용한다.
    */
    flecs::world& GetWorld();
    /*
        이 Scene에 속하는 모든 Entity의 최상위 부모.
    */
    flecs::entity GetSceneRoot() const;


private:
    /*
        SceneRoot의 생성/삭제는 Scene 사용자가 직접 하지 않고
        SceneManager가 Scene 전환 시점에 관리한다.

        그래서 SceneManager만 접근할 수 있도록 friend로 둔다.
    */
    friend class SceneManager;
    /*
        Scene 활성화 직전에 호출.

            SceneManager
                ↓
            InitRoot()
                ↓
            OnEnter()
    */
    void InitRoot();
    /*
        Scene 종료 시 호출.

        SceneRoot는 이 Scene에서 생성된 모든 Entity의
        hierarchy 최상위 노드다.
    */
    void CleanupRoot();
protected:
    /*
        non-owning reference.

        Scene은 Flecs World의 lifetime을 책임지지 않는다.
    */
    flecs::world& m_World;
    /*
        Scene의 hierarchy root.

        실제 공간 Transform을 가지는 RobotRoot와는 별개다.

        SceneRoot의 목적은 Scene 단위 lifetime 관리다.
    */
    flecs::entity m_SceneRoot{flecs::entity::null()};
};