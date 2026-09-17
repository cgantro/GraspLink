#pragma once

#include "Protocol.h"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace PoseLink
{
/**
 * @brief PC와 ESP32 사이 USB-UART 연결을 여는 데 필요한 불변 설정이다.
 *
 * `device`는 Windows에서는 `COM3` 또는 `\\\\.\\COM3`, POSIX에서는
 * `/dev/ttyUSB0` 같은 장치 경로다. `baudRate`는 양 끝단에서 반드시 같아야
 * 하며, 기본값 115200 bit/s는 ESP32 DevKitC의 console UART에 흔히 쓰이는
 * 속도다. 이 설정은 IP 주소나 포트가 없다는 점을 제외하면 UDP endpoint와
 * 같은 역할을 한다. USB-UART는 byte stream이므로 한 번의 Write가 한 번의
 * Read 또는 한 protocol packet과 일치한다는 보장은 없다.
 */
struct SerialPortSettings
{
    std::string device;
    std::uint32_t baudRate = 115200U;
};

/**
 * @brief non-blocking USB-UART byte-stream의 move-only RAII 소유자다.
 *
 * 이 객체 하나만 OS serial handle을 소유한다. 소멸, Close, move assignment의
 * 모든 경로에서 handle을 정확히 한 번 닫는다. Read는 현재 driver queue에 있는
 * 바이트만 반환하고, 없으면 `nullopt`를 반환하므로 simulator render loop를
 * 기다리게 하지 않는다.
 *
 * Write는 OS가 즉시 받을 수 없는 경우에도 caller를 block하지 않는다. `Send`
 * 는 작은 GLNK packet을 내부 FIFO에 넣고 전송 가능한 만큼만 진행한다. 따라서
 * caller는 packet을 재전송하거나 분할해서 보내지 않아야 한다. 큐가 4 KiB를
 * 넘으면 false를 반환하며, 이는 feedback producer가 무한히 메모리를 점유하지
 * 않도록 하는 back-pressure 경계다. `Pump`는 frame마다 호출해 pending write를
 * 계속 진전시킬 수 있다.
 */
class SerialTransport final
{
public:
    // Out-of-line definition keeps PlatformState opaque to users of this
    // header; unique_ptr may then own the OS-specific type safely.
    SerialTransport();
    ~SerialTransport();
    SerialTransport(const SerialTransport&) = delete;
    SerialTransport& operator=(const SerialTransport&) = delete;
    SerialTransport(SerialTransport&& other) noexcept;
    SerialTransport& operator=(SerialTransport&& other) noexcept;

    /** Opens, configures 8-N-1, and clears stale driver input/output bytes. */
    [[nodiscard]] bool Open(const SerialPortSettings& settings, std::string* error = nullptr);
    /** Queues one complete protocol packet without blocking the calling thread. */
    [[nodiscard]] bool Send(const std::vector<std::uint8_t>& bytes, std::string* error = nullptr);
    /** Advances queued output once; no data being writable is not an error. */
    [[nodiscard]] bool Pump(std::string* error = nullptr);
    /** Returns one currently available raw stream chunk, or nullopt when idle. */
    [[nodiscard]] std::optional<std::vector<std::uint8_t>> Receive(std::string* error = nullptr);

    [[nodiscard]] bool IsOpen() const noexcept;
    void Close() noexcept;

private:
    struct PlatformState;

#ifdef _WIN32
    using NativeHandle = void*;
    static constexpr NativeHandle kInvalidHandle = nullptr;
#else
    using NativeHandle = int;
    static constexpr NativeHandle kInvalidHandle = -1;
#endif

    [[nodiscard]] bool StartOrContinueWrite(std::string* error);
    void MoveFrom(SerialTransport&& other) noexcept;

    NativeHandle handle_ = kInvalidHandle;
    std::unique_ptr<PlatformState> platform_;
    std::deque<std::vector<std::uint8_t>> pendingWrites_;
    std::size_t pendingWriteOffset_ = 0U;
    std::size_t queuedWriteBytes_ = 0U;
};

/**
 * @brief stream에서 완전하고 검증된 GLNK packet만 잘라 내는 parser의 진단값이다.
 *
 * `discardedNoiseBytes`는 magic 이전의 console log, reset banner, 손상된 byte를
 * 뜻한다. `malformedPackets`는 GLNK magic을 찾았지만 version/type/reserved/float
 * 검증에 실패한 후보 수다. 두 값 모두 object, IK, grasp state 변경을 유발하지
 * 않는다.
 */
struct SerialFramingStatistics
{
    std::uint64_t discardedNoiseBytes = 0U;
    std::uint64_t malformedPackets = 0U;
};

/** A decoded GLNK packet together with its exact original wire bytes. */
struct FramedSerialPacket
{
    DecodedPacket decoded;
    std::vector<std::uint8_t> bytes;
};

/**
 * @brief UART byte stream을 protocol datagram 경계로 복원한다.
 *
 * GLNK v1은 packet 길이를 header의 `type`으로 결정한다 (TargetCommand=28 byte,
 * RobotState=20 byte). Parser는 magic `GLNK`를 byte 단위로 찾아 partial read와
 * packet coalescing을 모두 처리한다. checksum이 없는 v1에서 malformed 후보는
 * 한 byte만 버리고 다시 magic을 찾는다. 이 방식은 byte drop 뒤의 다음 정상
 * packet을 가능한 한 잃지 않는 resynchronization 정책이다.
 */
class SerialPacketFramer final
{
public:
    /** Appends arbitrary stream bytes and returns every complete valid packet. */
    [[nodiscard]] std::vector<FramedSerialPacket> Push(const std::vector<std::uint8_t>& bytes);
    [[nodiscard]] const SerialFramingStatistics& Statistics() const noexcept { return statistics_; }
    void Reset() noexcept;

private:
    void TrimOversizedBuffer();

    std::vector<std::uint8_t> bufferedBytes_;
    SerialFramingStatistics statistics_{};
};

/** Latest-only command policy diagnostics for the serial controller input. */
struct SerialTargetReceiverStatistics
{
    std::uint64_t acceptedPackets = 0U;
    std::uint64_t stalePackets = 0U;
    std::uint64_t ignoredNonTargetPackets = 0U;
};

/**
 * @brief framed UART input에 UDP receiver와 같은 newest-only sequence 정책을 적용한다.
 *
 * `Poll`은 serial driver의 현재 queue를 모두 drain한 뒤 가장 최근의 새
 * TargetCommand 하나만 반환한다. unsigned sequence의 half-range rule
 * (`0 < candidate-last < 2^31`)은 `0xffffffff -> 0` rollover를 허용하고
 * duplicate/old command가 scene을 되돌리는 것을 막는다.
 */
class LatestSerialTargetReceiver final
{
public:
    explicit LatestSerialTargetReceiver(SerialTransport& transport) : transport_(transport) {}

    [[nodiscard]] std::optional<TargetCommand> Poll(std::string* transportError = nullptr);
    [[nodiscard]] const SerialTargetReceiverStatistics& Statistics() const noexcept { return statistics_; }
    [[nodiscard]] const SerialFramingStatistics& FramingStatistics() const noexcept
    {
        return framer_.Statistics();
    }
    [[nodiscard]] std::optional<std::uint32_t> LastAcceptedSequence() const noexcept
    {
        return lastSequence_;
    }
    [[nodiscard]] static bool IsStrictlyNewer(std::uint32_t candidate, std::uint32_t baseline) noexcept;

private:
    SerialTransport& transport_; // non-owning: transport must outlive this receiver.
    SerialPacketFramer framer_;
    SerialTargetReceiverStatistics statistics_{};
    std::optional<std::uint32_t> lastSequence_;
};
} // namespace PoseLink
