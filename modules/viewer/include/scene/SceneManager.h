#pragma once

#include "scene/Scene.h"

#include <flecs.h>

#include <memory>
#include <type_traits>
#include <utility>

/**
 * @brief 활성 Scene과 다음 전환 대상 Scene을 소유한다.
 * @details SceneManager는 Flecs World를 빌리고 Scene 객체만 소유한다. LoadScene()은 Scene 객체를
 * 즉시 만들지만 root와 OnEnter()는 다음 OnUpdate()까지 미룬다. 전환 때 기존 OnExit()와 root
 * 계층 삭제를 끝내고 새 root를 만든 뒤 OnEnter()를 호출한다. 이전 Entity handle은 root 삭제와
 * 함께 무효가 되므로 전환을 넘어 보관하지 않는다.
 * @note OnExit와 삭제 observer가 World 및 물리 자원을 참조할 수 있다. manager와 관련 System은
 * World와 PhysicsWorld보다 먼저 정리하며 GPU Component의 마지막 참조는 OpenGL Context가 살아 있을 때 해제한다.
 */
class SceneManager
{
public:
    /** @brief 외부 소유 Flecs World를 빌린다. @param world Scene 저장소로 쓸 World. */
    explicit SceneManager(flecs::world& world);

    /** @brief 활성 Scene의 OnExit와 root 정리를 마친 뒤 보유 Scene을 폐기한다. */
    ~SceneManager();

    /**
     * @brief 다음 OnUpdate()에서 활성화할 Scene을 예약한다.
     * @tparam T Scene에서 파생된 타입.
     * @param args T 생성자에 전달할 인자.
     * @details T 생성자는 즉시 실행된다. 다른 Scene으로 예약을 덮어쓰면 기존 예약 객체는
     * 활성화되지 않은 채 폐기되며 OnEnter()/OnExit()는 호출되지 않는다. root 생성과 OnEnter()는
     * 예약 Scene이 다음 OnUpdate()에서 활성화될 때 수행된다.
     */
    template<typename T, typename... Args>
    void LoadScene(Args&&... args)
    {
        static_assert(std::is_base_of_v<Scene, T>, "T must derive from Scene");

        m_NextScene = std::make_unique<T>(
            m_World,
            std::forward<Args>(args)...);
    }

    /** @brief 예약 전환을 적용하고 활성 Scene을 갱신한다. @param dt 경과 시간 [s]. */
    void OnUpdate(float dt);

    /**
     * @brief 활성 Scene을 빌린 포인터로 반환한다.
     * @return 활성 Scene 또는 활성 Scene이 없을 때 nullptr.
     * @details 포인터는 manager가 소유한다. 다음 전환이나 manager 소멸 이후 사용하지 않는다.
     */
    Scene* GetActiveScene() const;

private:
    // 외부 World를 빌린다. Scene의 OnExit와 root 정리가 World 파괴보다 먼저 끝나야 한다.
    flecs::world& m_World;

    // Scene 객체는 manager가 소유하고 root와 Entity는 빌린 World가 관리한다.
    std::unique_ptr<Scene> m_ActiveScene;

    // 생성자는 실행됐지만 다음 OnUpdate 활성화 전이므로 root와 OnEnter는 아직 없다.
    std::unique_ptr<Scene> m_NextScene;
};
