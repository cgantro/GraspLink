#pragma once

#include "scene/Scene.h"

#include <flecs.h>

#include <memory>
#include <type_traits>
#include <utility>

/**
 * @brief 현재 장면을 바꾸고 각 Scene 객체의 시작·종료와 수명을 관리한다.
 * @details SceneManager는 장면 물체 저장소인 Flecs World를 빌려 쓰고 Scene 객체만 소유한다. LoadScene()은 새 Scene 객체를 예약하지만 물체를 만들 root와 OnEnter()는 다음 OnUpdate()까지 준비하지 않는다.
 * 전환할 때 이전 Scene의 OnExit()를 호출하고 root 아래의 물체를 삭제한 뒤 새 root를 만들고 OnEnter()를 호출한다. 이전 Entity 참조는 root와 함께 대상 물체가 삭제되므로 전환 뒤 사용할 수 없다.

 * Entity 삭제 callback은 삭제된 물체에 연결된 Jolt 물체도 정리할 수 있으므로 SceneManager와 이 callback을 등록한 System은 World 및 PhysicsWorld보다 먼저 끝나야 한다.
 * Mesh나 Texture를 GPU에서 지우는 마지막 참조도 OpenGL context가 살아 있을 때 해제해야 한다.
 */
class SceneManager
{
public:
    /** @brief Scene Entity를 저장할 외부 소유 Flecs World를 빌려 사용한다.
     * @param world SceneManager보다 오래 살아 있어야 하는 World 참조.
     */
    explicit SceneManager(flecs::world& world);

    /** @brief 활성 Scene의 OnExit()를 호출하고 root 계층을 정리한 다음 Scene 객체를 폐기한다. */
    ~SceneManager();

    /**
     * @brief 다음 OnUpdate()에서 활성화할 Scene 객체를 만들고 예약한다.
     * @tparam T Scene을 상속해 만든 새 Scene의 구체 타입이다.
     * @param args 새 Scene 생성자에 전달할 인자다.
     * @details 새 Scene 생성자는 이 호출 중 실행되지만 바닥이나 로봇 물체를 붙일 root는 다음 OnUpdate()에서 만든다.
     * 그 전에 다른 Scene을 예약하면 이전 예약 객체는 활성화되지 않고 폐기되므로 OnEnter()나 OnExit()를 호출하지 않는다.
     */
    template<typename T, typename... Args>
    void LoadScene(Args&&... args)
    {
        static_assert(std::is_base_of_v<Scene, T>, "T must derive from Scene");

        m_NextScene = std::make_unique<T>(
            m_World,
            std::forward<Args>(args)...);
    }

    /** @brief 예약된 Scene 전환을 적용한 뒤 활성 Scene을 갱신한다.
     * @param dt 이전 갱신 이후 흐른 시간 [s].
     */
    void OnUpdate(float dt);

    /**
     * @brief 활성 Scene을 빌린 포인터로 반환한다.
     * @return 활성 Scene 또는 활성 Scene이 없을 때 nullptr.
     * @details 포인터는 manager가 소유한다. 다음 전환이나 manager 소멸 이후 사용하지 않는다.
     */
    Scene* GetActiveScene() const;

private:
    // World는 외부 소유이므로 이 참조를 통해 Scene만 저장한다. Scene의 OnExit와 root 정리가 World 파괴보다 먼저 끝나야 한다.
    flecs::world& m_World;

    // Scene 객체의 수명은 manager가 소유하지만, 그 root와 Entity 데이터는 빌린 Flecs World가 관리한다.
    std::unique_ptr<Scene> m_ActiveScene;

    // 예약 Scene 생성자는 이미 실행됐지만 활성화는 다음 OnUpdate()까지 미뤄져 root와 OnEnter()는 아직 준비되지 않았다.
    std::unique_ptr<Scene> m_NextScene;
};
