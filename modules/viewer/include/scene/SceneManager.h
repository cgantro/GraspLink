#pragma once

#include "scene/Scene.h"

#include <flecs.h>

#include <memory>
#include <type_traits>
#include <utility>

/**
 * @brief 현재 Scene과 다음 frame boundary에서 활성화할 Scene을 소유/전환하는 manager.
 *
 * @details
 * LoadScene()은 즉시 현재 Scene을 삭제하지 않고 `m_NextScene`에 예약한다.
 * 실제 교체는 OnUpdate() 시작 시 수행한다. Scene update 또는 Flecs iteration 도중 hierarchy를 제거하면
 * iterator/lifetime 문제가 생길 수 있기 때문이다.
 *
 * 전환 순서:
 * `Active::OnExit -> CleanupRoot -> Next를 Active로 이동 -> InitRoot -> OnEnter -> OnUpdate(dt)`
 *
 * `dt`는 rendering frame delta time [s]이며 Scene logic에 그대로 전달된다.
 *
 * @todo [FUTURE] async asset loading, transition event, loading state가 필요해지면 명시적 state machine으로 확장한다.
 */
class SceneManager
{
public:
    /**
     * @brief 모든 Scene이 공유할 Flecs World를 연결한다.
     * @param world ViewerApp이 소유하는 ECS World.
     * @note SceneManager는 World를 소유하지 않는다.
     */
    explicit SceneManager(flecs::world& world);

    /** @brief Active/Next Scene을 정리하고 각 Scene root lifetime cleanup을 수행한다. */
    ~SceneManager();

    /**
     * @brief 다음 frame boundary에서 활성화할 Scene을 생성해 예약한다.
     * @tparam T Scene을 상속한 구체 타입.
     * @tparam Args T 생성자에 전달할 추가 인자 타입.
     * @param args `T(world, args...)` 형태로 전달될 인자.
     *
     * @note 연속 호출하면 아직 활성화되지 않은 m_NextScene은 새 예약으로 교체된다.
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
     * @brief 예약된 Scene 전환을 frame boundary에서 처리한 뒤 active Scene을 갱신한다.
     * @param dt 이전 rendering frame 이후 경과 시간 [s].
     */
    void OnUpdate(float dt);

    /**
     * @brief 현재 active Scene의 non-owning pointer를 반환한다.
     * @return active Scene이 없으면 nullptr.
     * @warning 다음 Scene 전환 후 기존 pointer는 더 이상 유효하지 않을 수 있다.
     */
    Scene* GetActiveScene() const;

private:
    /** @brief Scene에 공유하는 non-owning Flecs World reference. */
    flecs::world& m_World;

    /** @brief 현재 active Scene의 unique ownership. */
    std::unique_ptr<Scene> m_ActiveScene;

    /** @brief 다음 OnUpdate frame boundary에 활성화할 예약 Scene의 unique ownership. */
    std::unique_ptr<Scene> m_NextScene;
};
