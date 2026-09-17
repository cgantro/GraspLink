#include "grasplink/controller/TargetInputController.h"

namespace grasplink::controller
{
TargetInputController::TargetInputController(const TargetInputConfiguration& configuration) noexcept
    : configuration_(configuration)
{
    // 0으로 나누지 않도록 invalid board configuration을 안전한 1-code range로 낮춘다.
    // 이것은 board 설정 오류를 숨기기 위한 보정이 아니라 firmware가 NaN을 송신하지
    // 않게 하는 최후의 경계다. bring-up 때 log/assert로 별도 검증한다.
    if (configuration_.adcMaximum == 0U) {
        configuration_.adcMaximum = 1U;
    }

    snapshot_.targetId = configuration_.targetId;
    snapshot_.x = configuration_.x.minimum;
    snapshot_.y = configuration_.y.minimum;
    snapshot_.z = configuration_.z.minimum;
}

void TargetInputController::OnAdcSample(const uint16_t rawCode) noexcept
{
    switch (selectedAxis_) {
    case TargetAxis::X:
        snapshot_.x = MapAdcToRange(rawCode, configuration_.adcMaximum, configuration_.x);
        break;
    case TargetAxis::Y:
        snapshot_.y = MapAdcToRange(rawCode, configuration_.adcMaximum, configuration_.y);
        break;
    case TargetAxis::Z:
        snapshot_.z = MapAdcToRange(rawCode, configuration_.adcMaximum, configuration_.z);
        break;
    }
}

ButtonAction TargetInputController::OnButtonReleased(const uint32_t heldMs) noexcept
{
    if (heldMs >= configuration_.longPressThresholdMs) {
        return ButtonAction::SendSnapshot;
    }

    selectedAxis_ = NextAxis(selectedAxis_);
    return ButtonAction::AxisChanged;
}

TargetSnapshot TargetInputController::Snapshot() const noexcept
{
    return snapshot_;
}

TargetAxis TargetInputController::SelectedAxis() const noexcept
{
    return selectedAxis_;
}

float TargetInputController::MapAdcToRange(const uint16_t rawCode, const uint16_t adcMaximum,
                                           const CoordinateRange& range) noexcept
{
    const uint16_t clampedCode = rawCode > adcMaximum ? adcMaximum : rawCode;
    const float normalized = static_cast<float>(clampedCode) / static_cast<float>(adcMaximum);
    return range.minimum + (range.maximum - range.minimum) * normalized;
}

TargetAxis TargetInputController::NextAxis(const TargetAxis axis) noexcept
{
    switch (axis) {
    case TargetAxis::X:
        return TargetAxis::Y;
    case TargetAxis::Y:
        return TargetAxis::Z;
    case TargetAxis::Z:
    default:
        return TargetAxis::X;
    }
}
} // namespace grasplink::controller
