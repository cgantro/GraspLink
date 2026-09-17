#include "grasplink/controller/SerialTransport.h"

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include <cstring>

namespace grasplink::controller
{
namespace
{
constexpr uint8_t kMagicBytes[] = {'G', 'L', 'N', 'K'};
constexpr uint8_t kVersionOffset = 4U;
constexpr uint8_t kTypeOffset = 5U;
constexpr uint8_t kReservedOffset = 6U;
constexpr uint8_t kSequenceOffset = 8U;
constexpr uint8_t kAckSequenceOffset = 12U;
constexpr uint8_t kStatusOffset = 16U;
} // namespace

SerialTransport::SerialTransport(const device* uart, FeedbackPresenter& feedback) noexcept
    : uart_(uart), feedback_(feedback)
{
}

bool SerialTransport::IsReady() const noexcept
{
    return uart_ != nullptr && device_is_ready(uart_);
}

bool SerialTransport::Send(const TargetSnapshot& snapshot) noexcept
{
    // ADC mapping은 finite 값을 만들지만, transport boundary에서도 NaN/Inf 전송을 막아
    // simulator의 IK 계산으로 비정상 값이 전파되지 않게 한다.
    if (!IsReady() || !IsFinite(snapshot.x) || !IsFinite(snapshot.y) || !IsFinite(snapshot.z)) {
        feedback_.OnRobotStatus(RobotFeedbackStatus::TransportError);
        return false;
    }

    uint8_t packet[kTargetCommandSize]{};
    packet[0U] = kMagicBytes[0U];
    packet[1U] = kMagicBytes[1U];
    packet[2U] = kMagicBytes[2U];
    packet[3U] = kMagicBytes[3U];
    packet[kVersionOffset] = kVersion;
    packet[kTypeOffset] = kTargetCommandType;
    // reserved bytes[6..7]와 unused byte는 zero-initialization으로 명시적으로 0이다.
    WriteU32(packet, kSequenceOffset, nextCommandSequence_);
    WriteU32(packet, 12U, snapshot.targetId);
    WriteFloat(packet, 16U, snapshot.x);
    WriteFloat(packet, 20U, snapshot.y);
    WriteFloat(packet, 24U, snapshot.z);

    // uart_poll_out은 byte가 UART FIFO/shift register에 들어갈 수 있을 때까지 기다린다.
    // 28 byte만 쓰므로 button worker를 길게 점유하지 않고, ISR에서 호출하지 않는다는
    // ControllerInterfaces의 contract도 만족한다.
    for (uint8_t byte : packet) {
        uart_poll_out(uart_, byte);
    }

    ++statistics_.sentCommands;
    ++nextCommandSequence_; // unsigned overflow는 protocol이 정의한 wrapping sequence다.
    feedback_.OnRobotStatus(RobotFeedbackStatus::TargetReceived);
    return true;
}

void SerialTransport::Poll() noexcept
{
    if (!IsReady()) {
        return;
    }

    unsigned char byte = 0U;
    while (uart_poll_in(uart_, &byte) == 0) {
        PushByte(static_cast<uint8_t>(byte));
    }
}

SerialTransportStatistics SerialTransport::Statistics() const noexcept
{
    return statistics_;
}

uint32_t SerialTransport::LastAcknowledgedTargetSequence() const noexcept
{
    return lastAcknowledgedTargetSequence_;
}

bool SerialTransport::IsFinite(const float value) noexcept
{
    // IEEE-754 exponent가 모두 1이면 NaN 또는 +/-Inf다. <cmath>와 libm 없이 검사해
    // 작은 embedded C++ build에서도 동일한 packet reject policy를 유지한다.
    static_assert(sizeof(float) == sizeof(uint32_t), "GLNK v1 requires 32-bit float");
    uint32_t bits = 0U;
    std::memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7F800000U) != 0x7F800000U;
}

bool SerialTransport::IsNewerSequence(const uint32_t candidate, const uint32_t reference) noexcept
{
    // 0 < (candidate - reference) < 2^31이면 modular time line에서 candidate가 newer다.
    // 정확히 2^31 차이는 어느 방향인지 정의할 수 없으므로 stale로 보수적으로 버린다.
    const uint32_t difference = candidate - reference;
    return difference != 0U && difference < 0x80000000U;
}

uint32_t SerialTransport::ReadU32(const uint8_t* bytes, const uint8_t offset) noexcept
{
    return (static_cast<uint32_t>(bytes[offset]) << 24U)
        | (static_cast<uint32_t>(bytes[offset + 1U]) << 16U)
        | (static_cast<uint32_t>(bytes[offset + 2U]) << 8U)
        | static_cast<uint32_t>(bytes[offset + 3U]);
}

void SerialTransport::WriteU32(uint8_t* bytes, const uint8_t offset, const uint32_t value) noexcept
{
    bytes[offset] = static_cast<uint8_t>(value >> 24U);
    bytes[offset + 1U] = static_cast<uint8_t>(value >> 16U);
    bytes[offset + 2U] = static_cast<uint8_t>(value >> 8U);
    bytes[offset + 3U] = static_cast<uint8_t>(value);
}

void SerialTransport::WriteFloat(uint8_t* bytes, const uint8_t offset, const float value) noexcept
{
    uint32_t bits = 0U;
    std::memcpy(&bits, &value, sizeof(bits));
    WriteU32(bytes, offset, bits);
}

bool SerialTransport::IsValidStatus(const uint8_t status) noexcept
{
    return status <= 4U;
}

RobotFeedbackStatus SerialTransport::ToFeedbackStatus(const uint8_t status) noexcept
{
    switch (status) {
    case 0U:
        return RobotFeedbackStatus::Idle;
    case 1U:
        return RobotFeedbackStatus::Moving;
    case 2U:
        return RobotFeedbackStatus::GraspSuccess;
    case 3U:
        return RobotFeedbackStatus::GraspFailed;
    case 4U:
    default:
        return RobotFeedbackStatus::TransportError;
    }
}

void SerialTransport::PushByte(const uint8_t byte) noexcept
{
    if (frameSize_ == kTargetCommandSize) {
        // 이전 frame이 끝나지 않았다는 것은 손상/누락을 뜻한다. 이번 byte를 새 후보로
        // 취급해 동기화를 회복한다.
        ++statistics_.malformedFrames;
        ResetFrame();
    }
    frame_[frameSize_++] = byte;

    // 수신 도중 다음 GLNK가 보이면 이전 frame은 잘린 것이다. 새로운 magic부터 즉시
    // 다시 시작하면 케이블 연결 직후의 문자열이나 byte 유실 뒤에도 다음 state를 잃지 않는다.
    for (uint8_t start = 1U; start + 4U <= frameSize_; ++start) {
        if (frame_[start] == kMagicBytes[0U] && frame_[start + 1U] == kMagicBytes[1U]
            && frame_[start + 2U] == kMagicBytes[2U] && frame_[start + 3U] == kMagicBytes[3U]) {
            ++statistics_.malformedFrames;
            const uint8_t remaining = frameSize_ - start;
            for (uint8_t index = 0U; index < remaining; ++index) {
                frame_[index] = frame_[start + index];
            }
            frameSize_ = remaining;
            expectedFrameSize_ = 0U;
            break;
        }
    }

    if (frameSize_ >= 4U
        && (frame_[0U] != kMagicBytes[0U] || frame_[1U] != kMagicBytes[1U]
            || frame_[2U] != kMagicBytes[2U] || frame_[3U] != kMagicBytes[3U])) {
        Resynchronise();
        return;
    }

    if (expectedFrameSize_ == 0U && frameSize_ >= 6U) {
        if (frame_[kTypeOffset] == kTargetCommandType) {
            expectedFrameSize_ = kTargetCommandSize;
        } else if (frame_[kTypeOffset] == kRobotStateType) {
            expectedFrameSize_ = kRobotStateSize;
        } else {
            ++statistics_.malformedFrames;
            Resynchronise();
            return;
        }
    }

    if (expectedFrameSize_ != 0U && frameSize_ == expectedFrameSize_) {
        ProcessCompleteFrame();
        ResetFrame();
    }
}

void SerialTransport::ProcessCompleteFrame() noexcept
{
    const bool validHeader = frame_[kVersionOffset] == kVersion && frame_[kReservedOffset] == 0U
        && frame_[kReservedOffset + 1U] == 0U;
    if (!validHeader) {
        ++statistics_.malformedFrames;
        return;
    }

    if (frame_[kTypeOffset] == kTargetCommandType) {
        // Controller가 받는 TargetCommand는 echo/잘못된 PC endpoint다. UI state를 바꾸지 않는다.
        ++statistics_.ignoredTargetCommands;
        return;
    }

    if (frame_[kTypeOffset] != kRobotStateType || frameSize_ != kRobotStateSize
        || frame_[17U] != 0U || frame_[18U] != 0U || frame_[19U] != 0U
        || !IsValidStatus(frame_[kStatusOffset])) {
        ++statistics_.malformedFrames;
        return;
    }

    const uint32_t stateSequence = ReadU32(frame_, kSequenceOffset);
    if (hasRobotStateSequence_ && !IsNewerSequence(stateSequence, lastRobotStateSequence_)) {
        ++statistics_.staleStates;
        return;
    }

    hasRobotStateSequence_ = true;
    lastRobotStateSequence_ = stateSequence;
    lastAcknowledgedTargetSequence_ = ReadU32(frame_, kAckSequenceOffset);
    ++statistics_.receivedStates;
    feedback_.OnRobotStatus(ToFeedbackStatus(frame_[kStatusOffset]));
}

void SerialTransport::Resynchronise() noexcept
{
    // 현재 bytes 중 다음 packet의 magic prefix가 될 수 있는 가장 긴 suffix만 보존한다.
    // 예: "noiseGLN" 뒤에 K가 도착하면 GLNK를 정상 frame 시작으로 복원할 수 있다.
    uint8_t keep = 0U;
    const uint8_t maximum = frameSize_ < 3U ? frameSize_ : 3U;
    for (uint8_t candidate = maximum; candidate > 0U; --candidate) {
        bool matches = true;
        for (uint8_t index = 0U; index < candidate; ++index) {
            if (frame_[frameSize_ - candidate + index] != kMagicBytes[index]) {
                matches = false;
                break;
            }
        }
        if (matches) {
            keep = candidate;
            break;
        }
    }
    for (uint8_t index = 0U; index < keep; ++index) {
        frame_[index] = frame_[frameSize_ - keep + index];
    }
    frameSize_ = keep;
    expectedFrameSize_ = 0U;
}

void SerialTransport::ResetFrame() noexcept
{
    frameSize_ = 0U;
    expectedFrameSize_ = 0U;
}
} // namespace grasplink::controller
