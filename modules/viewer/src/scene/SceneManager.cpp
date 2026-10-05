#include "scene/SceneManager.h"


SceneManager::SceneManager(flecs::world& world)
    : m_World(world)
{
}


SceneManager::~SceneManager()
{
    // borrowed World와 삭제 observer가 살아 있을 때 종료 hook과 자손 삭제를 마친다.
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
    // 예약은 프레임 경계에 반영한다. LoadScene 호출 시점에는 활성 계층이 바뀌지 않는다.
    if (m_NextScene)
    {
        if (m_ActiveScene)
        {
            // OnExit에서 사용자 자원을 정리한 뒤 root 파괴로 Entity와 Component를 제거한다.
            m_ActiveScene->OnExit();
            m_ActiveScene->CleanupRoot();
        }

        // 이전 root 삭제 뒤 manager 소유권을 예약 Scene으로 옮긴다.
        m_ActiveScene =
            std::move(m_NextScene);
        // CreateEntity가 요구하는 활성 root를 먼저 준비하고 OnEnter에서 계층을 구성한다.
        m_ActiveScene->InitRoot();

        m_ActiveScene->OnEnter();
    }

    // 전환한 프레임에도 새 Scene의 첫 OnUpdate가 이어진다.
    if (m_ActiveScene)
    {
        m_ActiveScene->OnUpdate(dt);
    }
}


Scene* SceneManager::GetActiveScene() const
{
    return m_ActiveScene.get();
}
