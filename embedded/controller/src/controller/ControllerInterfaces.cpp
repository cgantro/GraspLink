#include "grasplink/controller/ControllerInterfaces.h"

namespace grasplink::controller
{
ButtonDispatchResult TargetInputWorkflow::OnButtonReleased(const uint32_t heldMs) noexcept
{
    const ButtonAction action = input_.OnButtonReleased(heldMs);
    if (action != ButtonAction::SendSnapshot) {
        return ButtonDispatchResult{action, false};
    }

    // `Snapshot()`은 input_ 내부를 가리키는 view가 아니라 값 복사본을 반환한다.
    // 따라서 long press 경계의 정확한 XYZ가 sender로 넘어가며, 이후의 ADC work가
    // 이미 시작한 송신 payload를 바꾸지 못한다.
    const TargetSnapshot snapshot = input_.Snapshot();
    return ButtonDispatchResult{action, sender_.Send(snapshot)};
}
} // namespace grasplink::controller
