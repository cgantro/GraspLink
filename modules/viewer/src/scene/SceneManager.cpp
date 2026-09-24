#include "scene/SceneManager.h"


SceneManager::SceneManager(flecs::world& world)
    : m_World(world)
{
}


SceneManager::~SceneManager()
{
    /*
        SceneManager가 제거될 때 Active Scene이 남아 있다면
        정상적인 Scene 종료 순서를 수행한다.

        ViewerApp에서는 반드시 flecs::world가 reset되기 전에
        SceneManager가 먼저 파괴되어야 한다.

        즉 lifetime은:

            SceneManager cleanup
                ↓
            flecs::world reset

        순서여야 한다.
    */
    if (m_ActiveScene)
    {
        m_ActiveScene->OnExit();
        m_ActiveScene->CleanupRoot();
    }


    m_NextScene.reset();
    m_ActiveScene.reset();
}


void SceneManager::OnUpdate(float dt)
{
    /*
        ========================================================================
        Scene Transition
        ========================================================================

        LoadScene<T>()가 호출되어 NextScene이 존재한다면
        frame 시작 시 Scene을 교체한다.
    */
    if (m_NextScene)
    {
        /*
            기존 Scene이 존재하면 먼저 종료한다.
        */
        if (m_ActiveScene)
        {
            /*
                Scene 고유 종료 처리.

                이후 Robot Scene에서는:

                    simulation state 정리
                    temporary state 정리

                등이 들어갈 수 있다.
            */
            m_ActiveScene->OnExit();
            /*
                SceneRoot를 제거해서
                기존 Scene hierarchy를 정리한다.
            */
            m_ActiveScene->CleanupRoot();
        }


        /*
            unique_ptr ownership 이동.

            std::move를 사용하는 이유:

                unique_ptr은 하나의 owner만 허용하므로
                복사할 수 없다.

            NextScene의 ownership을 ActiveScene으로 이동한다.
        */
        m_ActiveScene =
            std::move(m_NextScene);
        /*
            새 Scene의 hierarchy root를 먼저 만든다.

            반드시 OnEnter 이전이어야 한다.

            OnEnter에서:

                CreateEntity("Robot")

            을 호출하면 생성된 Entity가 즉시 SceneRoot의
            Child가 되어야 하기 때문이다.
        */
        m_ActiveScene->InitRoot();

        /*
            이제 Scene의 실제 Entity를 생성할 수 있다.
        */
        m_ActiveScene->OnEnter();
    }


    /*
        ========================================================================
        Active Scene Update
        ========================================================================

        Scene 전환이 끝난 뒤 현재 Scene을 update한다.
    */
    if (m_ActiveScene)
    {
        m_ActiveScene->OnUpdate(dt);
    }
}


Scene* SceneManager::GetActiveScene() const
{
    /*
        ownership은 SceneManager가 계속 유지한다.

        외부에서는 Scene을 사용하기 위한
        non-owning pointer만 받는다.
    */
    return m_ActiveScene.get();
}