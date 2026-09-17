#pragma once

#include "grasplink/controller/TargetInputController.h"

namespace grasplink::controller
{
/**
 * @brief TargetSnapshot을 simulator로 넘기는 전송 경계.
 *
 * concrete `UdpControllerClient`는 sequence를 할당하고 protocol v1 bytes를 만든다.
 * 입력 상태기계가 socket을 직접 알지 않게 분리하여, UI/ADC test는 Wi-Fi나 Zephyr
 * network stack 없이도 수행할 수 있다. false는 transport가 snapshot을 받지 못했음을
 * 뜻하며 caller는 UI에 재시도 상태를 표시할 수 있다.
 */
class ITargetCommandSender
{
public:
    virtual ~ITargetCommandSender() = default;
    virtual bool Send(const TargetSnapshot& snapshot) noexcept = 0;
};

/** @brief hardware/network binding 전 bring-up 및 host test에서 쓰는 무동작 sender. */
class NullTargetCommandSender final : public ITargetCommandSender
{
public:
    bool Send(const TargetSnapshot&) noexcept override { return false; }
};

/**
 * @brief button action과 transport 결과를 caller에게 함께 돌려주는 값 객체.
 *
 * `sendAccepted`는 UDP delivery 또는 simulator의 IK/grasp 성공이 아니라 sender가
 * snapshot을 받아 encode/send를 시작할 수 있었는지만 뜻한다. UDP 자체에는 delivery
 * acknowledgement가 없으므로, 실제 처리 여부는 이후 RobotState의 acknowledged
 * target sequence로 표시해야 한다.
 */
struct ButtonDispatchResult
{
    ButtonAction action = ButtonAction::AxisChanged;
    bool sendAccepted = false;
};

/**
 * @brief input state machine과 command sender를 연결하는 worker-context facade.
 *
 * 이 class가 존재하면 Zephyr callback은 ADC/button의 물리적 의미를 core로 전달하는
 * 데만 집중할 수 있다. long press 때 snapshot을 먼저 값으로 복사한 후 sender에
 * 전달하므로, 전송 중 다음 ADC sample이 생겨도 송신 대상이 바뀌지 않는다.
 */
class TargetInputWorkflow final
{
public:
    TargetInputWorkflow(TargetInputController& input, ITargetCommandSender& sender) noexcept
        : input_(input), sender_(sender)
    {
    }

    void OnAdcSample(uint16_t rawCode) noexcept { input_.OnAdcSample(rawCode); }

    ButtonDispatchResult OnButtonReleased(uint32_t heldMs) noexcept;

private:
    TargetInputController& input_;
    ITargetCommandSender& sender_;
};
} // namespace grasplink::controller
