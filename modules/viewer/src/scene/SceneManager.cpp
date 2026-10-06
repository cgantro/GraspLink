#include "scene/SceneManager.h"


SceneManager::SceneManager(flecs::world& world)
    : m_World(world)
{
}


SceneManager::~SceneManager()
{
    // observer는 Scene 물체 삭제를 알아채 Jolt 물체도 지우는 callback이다. 외부에서 빌린 World가 살아 있는 동안 Scene 종료와 자손 삭제를 마친다.
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
    // LoadScene()은 다음 Scene 객체만 예약한다. 현재 장면 물체는 이 OnUpdate()가 시작될 때까지 그대로 유지된다.
    if (m_NextScene)
    {
        if (m_ActiveScene)
        {
            // 먼저 OnExit()에서 Scene이 따로 가진 자원을 정리한 뒤 root를 지워 그 아래의 물체와 위치 같은 값도 제거한다.
            m_ActiveScene->OnExit();
            m_ActiveScene->CleanupRoot();
        }

        // 이전 장면 물체가 정리된 뒤 예약한 Scene 객체가 활성 Scene의 소유자가 된다.
        m_ActiveScene =
            std::move(m_NextScene);
        // OnEnter()에서 바닥과 로봇을 만들 수 있도록 새 root를 준비한 다음 활성화 함수를 호출한다.
        m_ActiveScene->InitRoot();

        m_ActiveScene->OnEnter();
    }

    // 전환을 적용한 프레임에 새 Scene의 첫 상태 갱신도 실행한다.
    if (m_ActiveScene)
    {
        m_ActiveScene->OnUpdate(dt);
    }
}


Scene* SceneManager::GetActiveScene() const
{
    return m_ActiveScene.get();
}
