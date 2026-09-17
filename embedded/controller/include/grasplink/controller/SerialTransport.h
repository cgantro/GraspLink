#pragma once

#include "grasplink/controller/ControllerInterfaces.h"
#include "grasplink/controller/FeedbackPresenter.h"

#include <stdint.h>

struct device;

namespace grasplink::controller
{
/**
 * @brief USB-UART byte stream 위에서 GLNK v1 packet 경계를 복원하는 고정 크기 수신기.
 *
 * UART는 UDP와 달리 packet 경계를 보존하지 않는 연속 byte stream이다. 따라서 송신측의
 * 28-byte TargetCommand와 수신측의 20-byte RobotState는 각각의 `write()` 호출 단위로
 * 구분할 수 없다. 이 수신기는 `GLNK` magic을 탐색하고 header의 type으로 정확한 frame
 * 길이를 결정한다. 전원이 켜질 때의 ROM log, 케이블 재연결, 손상된 byte는 다음 magic에서
 * 다시 동기화한다. heap이나 ISR callback을 사용하지 않으므로 모든 storage의 lifetime은
 * SerialTransport 객체와 같고, `Poll()`은 work/thread context에서만 호출해야 한다.
 */
struct SerialTransportStatistics
{
    uint32_t sentCommands = 0U;
    uint32_t receivedStates = 0U;
    uint32_t malformedFrames = 0U;
    uint32_t staleStates = 0U;
    uint32_t ignoredTargetCommands = 0U;
};

/**
 * @brief ESP32 USB-UART0와 simulator 사이의 GLNK v1 양방향 transport.
 *
 * Wire format은 UDP v1과 의도적으로 완전히 같다. 즉 integer와 IEEE-754 float bit pattern은
 * network byte order(big-endian)이고 C++ struct padding을 전송하지 않는다. transport만
 * UART로 바뀌므로 PC simulator와 controller core의 target/state 의미는 변하지 않는다.
 *
 * `Send()`는 caller가 long press에서 확정한 Cartesian snapshot에 sender-local, wrapping
 * sequence를 붙여 TargetCommand를 전송한다. `Poll()`은 RobotState sequence를 modular
 * half-range 규칙으로 비교해 duplicate/old state를 FeedbackPresenter에 전달하지 않는다.
 * 반환이 없는 Poll의 오류는 statistics와 `TransportError` feedback으로 관찰한다.
 */
class SerialTransport final : public ITargetCommandSender
{
public:
    static constexpr uint32_t kMagic = 0x474C4E4BU; // ASCII "GLNK"
    static constexpr uint8_t kVersion = 1U;
    static constexpr uint8_t kTargetCommandType = 1U;
    static constexpr uint8_t kRobotStateType = 2U;
    static constexpr uint8_t kTargetCommandSize = 28U;
    static constexpr uint8_t kRobotStateSize = 20U;

    /** `uart`는 caller가 소유하는 Zephyr UART device이며 transport 수명 동안 유효해야 한다. */
    SerialTransport(const device* uart, FeedbackPresenter& feedback) noexcept;

    /** @brief UART device가 준비됐는지 확인한다. 준비되지 않았으면 Send/Poll은 I/O를 하지 않는다. */
    [[nodiscard]] bool IsReady() const noexcept;

    /** @brief TargetSnapshot을 GLNK TargetCommand로 encode해 USB-UART에 순서대로 쓴다. */
    bool Send(const TargetSnapshot& snapshot) noexcept override;

    /** @brief 현재 UART FIFO의 모든 가용 byte를 읽고, 완전한 RobotState를 UI에 반영한다. */
    void Poll() noexcept;

    [[nodiscard]] SerialTransportStatistics Statistics() const noexcept;
    [[nodiscard]] uint32_t LastAcknowledgedTargetSequence() const noexcept;

private:
    static bool IsFinite(float value) noexcept;
    static bool IsNewerSequence(uint32_t candidate, uint32_t reference) noexcept;
    static uint32_t ReadU32(const uint8_t* bytes, uint8_t offset) noexcept;
    static void WriteU32(uint8_t* bytes, uint8_t offset, uint32_t value) noexcept;
    static void WriteFloat(uint8_t* bytes, uint8_t offset, float value) noexcept;
    static bool IsValidStatus(uint8_t status) noexcept;
    static RobotFeedbackStatus ToFeedbackStatus(uint8_t status) noexcept;

    void PushByte(uint8_t byte) noexcept;
    void ProcessCompleteFrame() noexcept;
    void Resynchronise() noexcept;
    void ResetFrame() noexcept;

    const device* uart_ = nullptr;
    FeedbackPresenter& feedback_;
    uint32_t nextCommandSequence_ = 1U;
    uint32_t lastAcknowledgedTargetSequence_ = 0U;
    uint32_t lastRobotStateSequence_ = 0U;
    bool hasRobotStateSequence_ = false;
    uint8_t frame_[kTargetCommandSize]{};
    uint8_t frameSize_ = 0U;
    uint8_t expectedFrameSize_ = 0U;
    SerialTransportStatistics statistics_{};
};
} // namespace grasplink::controller
