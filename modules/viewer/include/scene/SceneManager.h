#pragma once

#include "scene/Scene.h"

#include <flecs.h>

#include <memory>
#include <type_traits>
#include <utility>

/**
 * @brief 현재 Scene과 다음 frame에 전환할 Scene을 소유/관리한다.
 *
 * @details
 * LoadScene()은 즉시 현재 Scene을 삭제하지 않고 m_NextScene에 예약한다.
 * 실제 교체는 OnUpdate()의 frame boundary에서 수행한다. Scene update나 ECS iteration 도중
 * hierarchy를 제거하면 iterator/lifetime 문제가 생길 수 있기 때문이다.
 *
 * 전환 순서:
 * Active::OnExit -> CleanupRoot -> Next를 Active로 이동 -> InitRoot -> OnEnter
 *
 * @todo [FUTURE] Scene transition event나 async asset loading이 필요해지면 상태 머신으로 확장한다.
 */
class SceneManager
{
public:
    /** @brief Scene들이 공유할 Flecs World를 연결한다. */
    explicit SceneManager(flecs::world& world);

    /** @brief 소유 중인 Scene을 정리한다. */
    ~SceneManager();

    /**
     * @brief 다음 frame boundary에서 활성화할 Scene을 예약한다.
     * @tparam T Scene 파생 타입.
     * @tparam Args Scene 생성자에 전달할 추가 인자 타입.
     * @param args T 생성자에서 world 뒤에 전달할 인자.
     */
    template<typename T, typename... Args>
    void LoadScene(Args&&... args)
    {
        static_assert(std::is_base_of_v<Scene, T>, "T must derive from Scene");

        m_NextScene = std::make_unique<T>(
            m_World,
            std::forward<Args>(args)...);
    }

    /**
     * @brief 예약된 Scene 전환을 처리한 뒤 active Scene을 update한다.
     * @param dt frame delta time(second).
     */
    void OnUpdate(float dt);

    /**
     * @brief 현재 active Scene의 non-owning pointer를 반환한다.
     * @return active Scene이 없으면 nullptr.
     */
    Scene* GetActiveScene() const;

private:
    flecs::world& m_World;
    std::unique_ptr<Scene> m_ActiveScene;
    std::unique_ptr<Scene> m_NextScene;
};
