#pragma once

#include "scene/Scene.h"

#include <flecs.h>

#include <memory>
#include <type_traits>
#include <utility>


/*
    ============================================================================
    SceneManager
    ============================================================================

    현재 활성화된 Scene과 다음에 전환할 Scene을 관리한다.


    Scene 전환 흐름:

        LoadScene<T>()
              ↓
        m_NextScene

              ↓ 다음 OnUpdate

        ActiveScene::OnExit()
              ↓
        ActiveScene::CleanupRoot()

              ↓
        Next → Active

              ↓
        ActiveScene::InitRoot()
              ↓
        ActiveScene::OnEnter()


    Scene을 LoadScene() 즉시 교체하지 않고
    Next Scene에 예약해 두는 이유가 중요하다.

    Scene의 OnUpdate() 도중 현재 Scene hierarchy를 삭제하면
    현재 실행 중인 로직 또는 ECS iteration과 충돌할 수 있다.

    따라서 Scene 전환을 명시적인 frame 경계에서 수행한다.
*/
class SceneManager
{
public:
    explicit SceneManager(flecs::world& world);
    /*
        SceneManager가 Scene의 유일한 owner가 된다.

        따라서 shared ownership이 필요하지 않아 unique_ptr을 사용한다.

        RobotPal은 shared_ptr을 사용하지만 현재 MiniBCG에서는
        Scene을 외부에서 공유 소유할 이유가 없으므로 unique_ptr이
        더 명확한 ownership을 표현한다.
    */
    ~SceneManager();
    /*
        =========================================================================
        LoadScene
        =========================================================================

        즉시 Scene을 바꾸는 함수가 아니다.

        다음 Scene을 만들어 m_NextScene에 예약한다.

        실제 전환은 OnUpdate() 시작 시 수행된다.


        예:

            sceneManager.LoadScene<MyScene>();


        T는 반드시 Scene을 상속해야 한다.
    */
    template<typename T, typename... Args>
    void LoadScene(Args&&... args)
    {
        static_assert(
            std::is_base_of_v<Scene, T>,
            "T must derive from Scene"
        );


        /*
            Scene constructor의 첫 번째 인자는 항상 flecs::world&.

            추가 생성자 인자가 있다면 Args로 전달한다.

            예:

                LoadScene<MyScene>(someResource);

            →
                MyScene(m_World, someResource)
        */
        m_NextScene =
            std::make_unique<T>(
                m_World,
                std::forward<Args>(args)...
            );
    }
    /*
        Scene 전환 처리 + 현재 Scene Update.
    */
    void OnUpdate(float dt);

    /*
        현재 활성화된 Scene 조회.

        SceneManager가 ownership을 가지고 있으므로
        호출자는 raw pointer만 받는다.

        반환 pointer를 delete하면 안 된다.
    */
    Scene* GetActiveScene() const;


private:
    /*
        SceneManager 역시 World를 소유하지 않는다.
    */
    flecs::world& m_World;
    /*
        현재 실행 중인 Scene.
    */
    std::unique_ptr<Scene> m_ActiveScene;

    /*
        다음 frame boundary에서 활성화할 Scene.
    */
    std::unique_ptr<Scene> m_NextScene;
};