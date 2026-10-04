#pragma once

#include <flecs.h>

/**
 * @brief Local TRS Component를 행렬로 변환하고 parent hierarchy를 따라 World Transform을 계산하는 Flecs module.
 *
 * @details
 * 현재 Transform 계산 규칙은 다음과 같다.
 *
 * `Local = Translation * Rotation * Scale`
 * `World = ParentWorld * Local`
 *
 * 데이터 의미:
 * - Position(Local): 부모 frame 기준 translation. controller-ready HCR scene에서는 meter [m].
 * - Rotation(Local): XYZ Euler angle [rad]. 내부에서 `glm::quat(rotation)`으로 quaternion을 만든다.
 * - Scale(Local): 무차원 축별 배율.
 * - TransformMatrix(Local/World): GLM column-major 4x4 homogeneous matrix.
 *
 * RobotTransformAdapter가 RobotState joint angle [rad]을 Joint Entity의 Local Rotation에 기록하면,
 * 이 System이 같은 frame에서 rotation matrix를 만들고 GLB parent-child hierarchy를 따라 모든 Link의 World matrix를 갱신한다.
 * 관절 pivot translation은 GLB의 Joint Node local translation에 이미 포함되어 있으므로 별도 pivot matrix를 추가하지 않는다.
 *
 * 부모부터 자식 순서로 World matrix가 필요하므로 World query는 Flecs parent().cascade()를 사용한다.
 *
 * @warning 현재 Euler Rotation 저장 때문에 복합 회전에서 표현 특이점/순서 문제가 존재할 수 있다.
 * @todo [FUTURE] quaternion Rotation Component로 전환해 robotics/graphics 사이의 Euler round-trip을 제거한다.
 * @todo [FUTURE] Entity 수가 커지면 매 frame 전체 계산 대신 dirty/observer 기반 갱신을 검토한다.
 */
class TransformSystemModule
{
public:
    /**
     * @brief World에 Local/World Transform 계산 System을 등록한다.
     * @param world Transform Component와 hierarchy를 보유하는 Flecs World.
     */
    explicit TransformSystemModule(flecs::world& world);

private:
    /**
     * @brief 향후 dirty-transform observer 확장을 위한 등록 지점.
     * @param world observer를 등록할 Flecs World.
     * @note 현재 구현은 매 frame 전체 transform을 계산하므로 no-op이다.
     */
    void RegisterObserver(flecs::world& world);

    /**
     * @brief `UpdateLocalTransform`과 `UpdateWorldTransform` Flecs System을 등록한다.
     * @param world system을 등록할 Flecs World.
     */
    void RegisterSystem(flecs::world& world);
};
