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
/*
 * [추가 그래픽스/ECS 용어 설명]
 * - TRS: Translation(이동), Rotation(회전), Scale(크기)의 약자.
 * - Local Matrix: 부모 좌표계에서 이 Entity가 어디에 있고 어떻게 회전/확대됐는지 나타내는 행렬.
 * - World Matrix: 부모의 World Transform까지 모두 누적해 Scene 전체 기준으로 계산한 최종 행렬.
 * - Hierarchy: Parent/Child로 연결된 Transform tree.
 * - Cascade: 부모부터 자식 순서가 보장되도록 ECS query를 실행하는 방식.
 * - Dirty Flag: 값이 실제로 바뀐 Entity만 다시 계산하기 위해 변경 여부를 기록하는 표시.
 * - Observer: Component 변경 event를 감지해 추가 처리를 실행하는 ECS 기능.
 *
 * 예: Joint Entity의 Local Rotation이 바뀌면 그 아래 Link/J2/... 자식들의 World Matrix도 부모 행렬을 통해 함께 달라진다.
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
