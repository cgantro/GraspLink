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
/*
 * [추가 용어 설명]
 * - Active Scene: 현재 update/render 대상이 되는 Scene.
 * - Next Scene: 다음 안전한 전환 시점에 Active가 될 예정인 Scene.
 * - Frame Boundary: 한 frame의 처리 중간이 아니라 다음 update를 시작하기 전처럼 상태를 바꾸기 안전한 경계.
 * - ECS Iteration: System이 조건에 맞는 Entity들을 순회하는 중인 상태.
 * - Iterator/Lifetime 문제: 순회 중 Entity hierarchy를 삭제하면 현재 순회가 가리키는 대상이 사라질 수 있는 문제.
 * - unique_ptr: Scene 하나의 명확한 소유권을 SceneManager가 가진다는 의미.
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
    // ViewerApp이 실제로 소유하는 Flecs World에 대한 non-owning reference.
    flecs::world& m_World;

    // 현재 활성 Scene의 단독 소유권.
    std::unique_ptr<Scene> m_ActiveScene;

    // 다음 frame boundary에서 활성화할 예약 Scene의 단독 소유권.
    std::unique_ptr<Scene> m_NextScene;
};
