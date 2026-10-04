#pragma once

#include <flecs.h>

/**
 * @brief Local TRS를 행렬로 만들고 부모 계층을 따라 World Transform을 계산하는 Flecs module.
 *
 * @details
 * Local Matrix = Translation * Rotation * Scale
 * World Matrix = Parent World * Local
 *
 * 부모부터 자식 순서로 계산해야 하므로 World 계산 query는 Flecs cascade를 사용한다.
 *
 * @todo [FUTURE] 현재는 매 frame 전체 Transform을 다시 계산한다. Entity 수가 커지면
 *       변경된 Transform만 갱신하는 dirty/observer 방식으로 최적화할 수 있다.
 */
class TransformSystemModule
{
public:
    /** @brief World에 Local/World Transform 계산 System을 등록한다. */
    explicit TransformSystemModule(flecs::world& world);

private:
    /** @brief Transform 변경 감시 확장을 위한 등록 지점. 현재 구현은 no-op이다. */
    void RegisterObserver(flecs::world& world);

    /** @brief Local matrix와 hierarchy 기반 World matrix 계산 System을 등록한다. */
    void RegisterSystem(flecs::world& world);
};
