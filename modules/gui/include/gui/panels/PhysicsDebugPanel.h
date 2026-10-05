#pragma once

namespace grasplink::gui
{

/**
 * @brief 설정 Collider 표시 여부를 조작하는 ImGui 패널.
 * @details 체크박스 상태만 소유한다. Collider 조회·투영은 ColliderOverlay가 담당하며,
 * 문구는 ECS 설정의 근사 표시와 100 ms 갱신 간격을 안내한다.
 */
class PhysicsDebugPanel
{
public:
    /** @brief 활성 ImGui 프레임 안에서 Collider 표시 체크박스와 범례를 그린다. */
    void Draw();

    /** @brief 체크박스에서 선택한 Collider 표시 여부를 반환한다. */
    [[nodiscard]] bool IsColliderVisible() const;

private:
    bool visible_ = false;
};

}
