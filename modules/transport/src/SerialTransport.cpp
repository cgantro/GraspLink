#include "SerialTransport.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstring>
#include <limits>
#include <utility>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace PoseLink
{
namespace
{
constexpr std::size_t kReadChunkBytes = 512U;
constexpr std::size_t kMaximumQueuedWriteBytes = 4096U;
constexpr std::size_t kMaximumFramerBufferedBytes = Protocol::kTargetCommandSize * 4U;
constexpr std::array<std::uint8_t, 4U> kMagicBytes{{'G', 'L', 'N', 'K'}};

void SetError(std::string* error, const std::string& message)
{
    if (error != nullptr) { *error = message; }
}

#ifdef _WIN32
HANDLE ToSystemHandle(void* handle) { return static_cast<HANDLE>(handle); }
void* FromSystemHandle(HANDLE handle) { return static_cast<void*>(handle); }

void SetWindowsError(std::string* error, const char* operation)
{
    SetError(error, std::string(operation) + " failed with Win32 error "
                    + std::to_string(static_cast<unsigned long>(GetLastError())));
}

std::string NormalizeWindowsDeviceName(const std::string& device)
{
    if (device.rfind("\\\\.\\", 0U) == 0U) { return device; }
    // Windows accepts COM1..COM9 directly, but the explicit device namespace
    // also makes COM10 and above unambiguous.
    return "\\\\.\\" + device;
}
#else
void SetPosixError(std::string* error, const char* operation)
{
    SetError(error, std::string(operation) + " failed: " + std::strerror(errno));
}

std::optional<speed_t> ToPosixBaud(std::uint32_t baudRate)
{
    switch (baudRate)
    {
    case 9600U: return B9600;
    case 19200U: return B19200;
    case 38400U: return B38400;
    case 57600U: return B57600;
    case 115200U: return B115200;
#ifdef B230400
    case 230400U: return B230400;
#endif
    default: return std::nullopt;
    }
}
#endif

std::optional<std::size_t> PacketSizeForType(std::uint8_t type)
{
    if (type == static_cast<std::uint8_t>(MessageType::TargetCommand))
    {
        return Protocol::kTargetCommandSize;
    }
    if (type == static_cast<std::uint8_t>(MessageType::RobotState))
    {
        return Protocol::kRobotStateSize;
    }
    return std::nullopt;
}
} // namespace

struct SerialTransport::PlatformState
{
#ifdef _WIN32
    HANDLE readEvent = nullptr;
    HANDLE writeEvent = nullptr;
    OVERLAPPED readOverlapped{};
    OVERLAPPED writeOverlapped{};
    std::array<std::uint8_t, kReadChunkBytes> readBuffer{};
    bool readPending = false;
    bool writePending = false;
#endif
};

SerialTransport::SerialTransport() = default;
SerialTransport::~SerialTransport() { Close(); }

SerialTransport::SerialTransport(SerialTransport&& other) noexcept { MoveFrom(std::move(other)); }

SerialTransport& SerialTransport::operator=(SerialTransport&& other) noexcept
{
    if (this != &other)
    {
        Close();
        MoveFrom(std::move(other));
    }
    return *this;
}

void SerialTransport::MoveFrom(SerialTransport&& other) noexcept
{
    handle_ = other.handle_;
    platform_ = std::move(other.platform_);
    pendingWrites_ = std::move(other.pendingWrites_);
    pendingWriteOffset_ = other.pendingWriteOffset_;
    queuedWriteBytes_ = other.queuedWriteBytes_;
    other.handle_ = kInvalidHandle;
    other.pendingWriteOffset_ = 0U;
    other.queuedWriteBytes_ = 0U;
}

bool SerialTransport::Open(const SerialPortSettings& settings, std::string* error)
{
    Close();
    if (settings.device.empty())
    {
        SetError(error, "Serial device must not be empty");
        return false;
    }
    if (settings.baudRate == 0U)
    {
        SetError(error, "Serial baud rate must be non-zero");
        return false;
    }

#ifdef _WIN32
    const std::string device = NormalizeWindowsDeviceName(settings.device);
    const HANDLE opened = CreateFileA(device.c_str(), GENERIC_READ | GENERIC_WRITE, 0,
                                      nullptr, OPEN_EXISTING,
                                      FILE_ATTRIBUTE_NORMAL | FILE_FLAG_OVERLAPPED, nullptr);
    if (opened == INVALID_HANDLE_VALUE)
    {
        SetWindowsError(error, "CreateFileA");
        return false;
    }

    DCB dcb{};
    dcb.DCBlength = sizeof(dcb);
    if (!GetCommState(opened, &dcb))
    {
        SetWindowsError(error, "GetCommState");
        CloseHandle(opened);
        return false;
    }
    dcb.BaudRate = static_cast<DWORD>(settings.baudRate);
    dcb.ByteSize = 8U;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    dcb.fBinary = TRUE;
    dcb.fParity = FALSE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fDtrControl = DTR_CONTROL_ENABLE;
    dcb.fRtsControl = RTS_CONTROL_ENABLE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    if (!SetCommState(opened, &dcb))
    {
        SetWindowsError(error, "SetCommState");
        CloseHandle(opened);
        return false;
    }
    COMMTIMEOUTS timeouts{};
    if (!SetCommTimeouts(opened, &timeouts) || !SetupComm(opened, 1024U, 1024U))
    {
        SetWindowsError(error, "serial configuration");
        CloseHandle(opened);
        return false;
    }
    auto platform = std::make_unique<PlatformState>();
    platform->readEvent = CreateEventA(nullptr, TRUE, FALSE, nullptr);
    platform->writeEvent = CreateEventA(nullptr, TRUE, FALSE, nullptr);
    if (platform->readEvent == nullptr || platform->writeEvent == nullptr)
    {
        SetWindowsError(error, "CreateEventA");
        if (platform->readEvent != nullptr) { CloseHandle(platform->readEvent); }
        if (platform->writeEvent != nullptr) { CloseHandle(platform->writeEvent); }
        CloseHandle(opened);
        return false;
    }
    platform->readOverlapped.hEvent = platform->readEvent;
    platform->writeOverlapped.hEvent = platform->writeEvent;
    PurgeComm(opened, PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR);
    handle_ = FromSystemHandle(opened);
    platform_ = std::move(platform);
#else
    const std::optional<speed_t> baud = ToPosixBaud(settings.baudRate);
    if (!baud.has_value())
    {
        SetError(error, "Unsupported POSIX serial baud rate");
        return false;
    }
    const int opened = open(settings.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (opened < 0)
    {
        SetPosixError(error, "open");
        return false;
    }
    termios tty{};
    if (tcgetattr(opened, &tty) != 0)
    {
        SetPosixError(error, "tcgetattr");
        close(opened);
        return false;
    }
    cfmakeraw(&tty);
    tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<tcflag_t>(~(PARENB | CSTOPB | CSIZE));
    tty.c_cflag |= CS8;
#ifdef CRTSCTS
    tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    if (cfsetispeed(&tty, *baud) != 0 || cfsetospeed(&tty, *baud) != 0
        || tcsetattr(opened, TCSANOW, &tty) != 0)
    {
        SetPosixError(error, "serial configuration");
        close(opened);
        return false;
    }
    handle_ = opened;
    platform_ = std::make_unique<PlatformState>();
#endif
    return true;
}

bool SerialTransport::Send(const std::vector<std::uint8_t>& bytes, std::string* error)
{
    if (!IsOpen())
    {
        SetError(error, "Send called on closed serial transport");
        return false;
    }
    if (bytes.empty())
    {
        SetError(error, "Serial packet must contain at least one byte");
        return false;
    }
    if (bytes.size() > kMaximumQueuedWriteBytes
        || queuedWriteBytes_ > kMaximumQueuedWriteBytes - bytes.size())
    {
        SetError(error, "Serial output queue is full");
        return false;
    }
    pendingWrites_.push_back(bytes);
    queuedWriteBytes_ += bytes.size();
    return Pump(error);
}

bool SerialTransport::Pump(std::string* error)
{
    if (!IsOpen())
    {
        SetError(error, "Pump called on closed serial transport");
        return false;
    }
    return StartOrContinueWrite(error);
}

bool SerialTransport::StartOrContinueWrite(std::string* error)
{
    if (pendingWrites_.empty()) { return true; }

#ifdef _WIN32
    PlatformState& state = *platform_;
    if (state.writePending)
    {
        DWORD completed = 0U;
        if (!GetOverlappedResult(ToSystemHandle(handle_), &state.writeOverlapped, &completed, FALSE))
        {
            if (GetLastError() == ERROR_IO_INCOMPLETE) { return true; }
            SetWindowsError(error, "GetOverlappedResult(write)");
            return false;
        }
        state.writePending = false;
        ResetEvent(state.writeEvent);
        pendingWriteOffset_ += static_cast<std::size_t>(completed);
    }
    if (pendingWriteOffset_ >= pendingWrites_.front().size())
    {
        queuedWriteBytes_ -= pendingWrites_.front().size();
        pendingWrites_.pop_front();
        pendingWriteOffset_ = 0U;
        if (pendingWrites_.empty()) { return true; }
    }
    DWORD written = 0U;
    const std::vector<std::uint8_t>& current = pendingWrites_.front();
    const DWORD remaining = static_cast<DWORD>(current.size() - pendingWriteOffset_);
    ResetEvent(state.writeEvent);
    if (WriteFile(ToSystemHandle(handle_), current.data() + pendingWriteOffset_, remaining,
                  &written, &state.writeOverlapped))
    {
        // A successful zero-byte completion is unusual for a configured COM
        // port, but treating it as "try on the next Pump" avoids unbounded
        // recursion if a driver reports temporary zero progress.
        if (written == 0U) { return true; }
        pendingWriteOffset_ += static_cast<std::size_t>(written);
        return StartOrContinueWrite(error);
    }
    if (GetLastError() == ERROR_IO_PENDING)
    {
        state.writePending = true;
        return true;
    }
    SetWindowsError(error, "WriteFile");
    return false;
#else
    const std::vector<std::uint8_t>& current = pendingWrites_.front();
    const std::size_t remaining = current.size() - pendingWriteOffset_;
    const ssize_t written = write(handle_, current.data() + pendingWriteOffset_, remaining);
    if (written < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK) { return true; }
        SetPosixError(error, "write");
        return false;
    }
    pendingWriteOffset_ += static_cast<std::size_t>(written);
    if (pendingWriteOffset_ == current.size())
    {
        queuedWriteBytes_ -= current.size();
        pendingWrites_.pop_front();
        pendingWriteOffset_ = 0U;
    }
    return true;
#endif
}

std::optional<std::vector<std::uint8_t>> SerialTransport::Receive(std::string* error)
{
    if (!IsOpen())
    {
        SetError(error, "Receive called on closed serial transport");
        return std::nullopt;
    }

#ifdef _WIN32
    PlatformState& state = *platform_;
    if (state.readPending)
    {
        DWORD completed = 0U;
        if (!GetOverlappedResult(ToSystemHandle(handle_), &state.readOverlapped, &completed, FALSE))
        {
            if (GetLastError() == ERROR_IO_INCOMPLETE) { return std::nullopt; }
            SetWindowsError(error, "GetOverlappedResult(read)");
            return std::nullopt;
        }
        state.readPending = false;
        ResetEvent(state.readEvent);
        if (completed == 0U) { return std::nullopt; }
        return std::vector<std::uint8_t>(state.readBuffer.begin(),
                                         state.readBuffer.begin() + completed);
    }
    DWORD received = 0U;
    ResetEvent(state.readEvent);
    if (ReadFile(ToSystemHandle(handle_), state.readBuffer.data(),
                 static_cast<DWORD>(state.readBuffer.size()), &received, &state.readOverlapped))
    {
        if (received == 0U) { return std::nullopt; }
        return std::vector<std::uint8_t>(state.readBuffer.begin(),
                                         state.readBuffer.begin() + received);
    }
    if (GetLastError() == ERROR_IO_PENDING)
    {
        state.readPending = true;
        return std::nullopt;
    }
    SetWindowsError(error, "ReadFile");
    return std::nullopt;
#else
    std::array<std::uint8_t, kReadChunkBytes> buffer{};
    const ssize_t received = read(handle_, buffer.data(), buffer.size());
    if (received < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK) { return std::nullopt; }
        SetPosixError(error, "read");
        return std::nullopt;
    }
    if (received == 0) { return std::nullopt; }
    return std::vector<std::uint8_t>(buffer.begin(), buffer.begin() + received);
#endif
}

bool SerialTransport::IsOpen() const noexcept { return handle_ != kInvalidHandle; }

void SerialTransport::Close() noexcept
{
    if (IsOpen())
    {
#ifdef _WIN32
        const HANDLE systemHandle = ToSystemHandle(handle_);
        CancelIoEx(systemHandle, nullptr);
        CloseHandle(systemHandle);
#else
        close(handle_);
#endif
        handle_ = kInvalidHandle;
    }
#ifdef _WIN32
    if (platform_)
    {
        if (platform_->readEvent != nullptr) { CloseHandle(platform_->readEvent); }
        if (platform_->writeEvent != nullptr) { CloseHandle(platform_->writeEvent); }
    }
#endif
    platform_.reset();
    pendingWrites_.clear();
    pendingWriteOffset_ = 0U;
    queuedWriteBytes_ = 0U;
}

std::vector<FramedSerialPacket> SerialPacketFramer::Push(const std::vector<std::uint8_t>& bytes)
{
    bufferedBytes_.insert(bufferedBytes_.end(), bytes.begin(), bytes.end());
    std::vector<FramedSerialPacket> packets;

    while (bufferedBytes_.size() >= kMagicBytes.size())
    {
        const auto magic = std::search(bufferedBytes_.begin(), bufferedBytes_.end(),
                                       kMagicBytes.begin(), kMagicBytes.end());
        if (magic == bufferedBytes_.end())
        {
            // The final three bytes can be the prefix of magic split across the
            // next USB read. Everything before them is irrecoverable noise.
            const std::size_t keep = kMagicBytes.size() - 1U;
            const std::size_t discard = bufferedBytes_.size() - keep;
            statistics_.discardedNoiseBytes += discard;
            bufferedBytes_.erase(bufferedBytes_.begin(), bufferedBytes_.begin() + discard);
            break;
        }
        if (magic != bufferedBytes_.begin())
        {
            const std::size_t discard = static_cast<std::size_t>(magic - bufferedBytes_.begin());
            statistics_.discardedNoiseBytes += discard;
            bufferedBytes_.erase(bufferedBytes_.begin(), magic);
            continue;
        }
        if (bufferedBytes_.size() < 6U) { break; }
        const std::optional<std::size_t> packetSize = PacketSizeForType(bufferedBytes_[5U]);
        if (!packetSize.has_value())
        {
            ++statistics_.malformedPackets;
            ++statistics_.discardedNoiseBytes;
            bufferedBytes_.erase(bufferedBytes_.begin());
            continue;
        }
        if (bufferedBytes_.size() < *packetSize) { break; }

        std::vector<std::uint8_t> candidate(bufferedBytes_.begin(),
                                            bufferedBytes_.begin() + *packetSize);
        const std::optional<DecodedPacket> decoded = Protocol::Decode(candidate);
        if (!decoded.has_value())
        {
            // Do not consume an assumed packet length. A byte loss can shift a
            // later magic into this candidate; one-byte resync preserves it.
            ++statistics_.malformedPackets;
            ++statistics_.discardedNoiseBytes;
            bufferedBytes_.erase(bufferedBytes_.begin());
            continue;
        }
        packets.push_back(FramedSerialPacket{*decoded, std::move(candidate)});
        bufferedBytes_.erase(bufferedBytes_.begin(), bufferedBytes_.begin() + *packetSize);
    }
    // A large USB read can legitimately contain many packets, so limiting the
    // buffer before the extraction loop would silently drop valid commands.
    // Only an incomplete/noisy suffix is bounded.
    TrimOversizedBuffer();
    return packets;
}

void SerialPacketFramer::Reset() noexcept
{
    bufferedBytes_.clear();
    statistics_ = {};
}

void SerialPacketFramer::TrimOversizedBuffer()
{
    if (bufferedBytes_.size() <= kMaximumFramerBufferedBytes) { return; }
    const std::size_t discard = bufferedBytes_.size() - kMaximumFramerBufferedBytes;
    statistics_.discardedNoiseBytes += discard;
    bufferedBytes_.erase(bufferedBytes_.begin(), bufferedBytes_.begin() + discard);
}

bool LatestSerialTargetReceiver::IsStrictlyNewer(
    std::uint32_t candidate, std::uint32_t baseline) noexcept
{
    const std::uint32_t delta = candidate - baseline;
    return delta != 0U && (delta & 0x80000000U) == 0U;
}

std::optional<TargetCommand> LatestSerialTargetReceiver::Poll(std::string* transportError)
{
    std::string pumpError;
    if (!transport_.Pump(&pumpError) && transportError != nullptr) { *transportError = pumpError; }

    std::optional<TargetCommand> newest;
    while (true)
    {
        std::string receiveError;
        const std::optional<std::vector<std::uint8_t>> chunk = transport_.Receive(&receiveError);
        if (!chunk.has_value())
        {
            if (!receiveError.empty() && transportError != nullptr) { *transportError = receiveError; }
            break;
        }
        for (const FramedSerialPacket& packet : framer_.Push(*chunk))
        {
            if (!packet.decoded.target.has_value())
            {
                ++statistics_.ignoredNonTargetPackets;
                continue;
            }
            const TargetCommand& candidate = *packet.decoded.target;
            if (lastSequence_.has_value() && !IsStrictlyNewer(candidate.sequence, *lastSequence_))
            {
                ++statistics_.stalePackets;
                continue;
            }
            lastSequence_ = candidate.sequence;
            newest = candidate;
            ++statistics_.acceptedPackets;
        }
    }
    return newest;
}
} // namespace PoseLink
