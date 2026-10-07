#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

namespace grasplink::diagnostics
{
/**
 * @brief 로그 기록의 심각도를 나타낸다.
 * @details 대기열이 가득 차면 Error 기록이 수치·성능·Debug·Info 기록을 밀어내고 들어갈 수 있다.
 */
enum class LogLevel
{
    Debug,
    Info,
    Warning,
    Error
};

/**
 * @brief Logger의 출력 파일과 제한된 기록 대기열 크기를 설정한다.
 * @details 출력 경로는 프로세스의 현재 작업 디렉터리를 기준으로 한다. 대기열 용량이 작을수록 메모리는 덜 쓰지만 기록이 더 많이 버려질 수 있다.
 */
struct LoggerOptions
{
    std::string filePath = "logs/grasplink.jsonl";
    std::size_t queueCapacity = 8192;
};

/**
 * @brief 로그, 수치와 성능 기록을 전용 작업 스레드에서 JSON Lines 파일로 저장한다.
 * @details 호출 스레드는 소유한 문자열과 수치만 제한된 대기열에 넣으며 파일 접근은 소비자 스레드만 수행한다.
 * 대기열이 가득 차면 새 기록을 버리고 누락 수를 세므로 제어 tick이 파일 쓰기 때문에 멈추지 않는다.
 * Shutdown은 이미 받은 기록을 파일에 쓰도록 모두 시도한 뒤 작업 스레드를 기다린다.
 */
class Logger final
{
public:
    explicit Logger(LoggerOptions options = {});
    ~Logger();

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    /**
     * @brief 분류와 메시지를 가진 로그 기록을 대기열에 추가한다.
     * @param level 기록의 심각도다.
     * @param category 오류가 발생한 시스템이나 기능의 이름이다.
     * @param message 파일에 저장할 소유 문자열이다.
     */
    void Write(LogLevel level, const std::string& category, const std::string& message) noexcept;

    /**
     * @brief 단위가 명시된 수치 기록을 대기열에 추가한다.
     * @param name 측정값의 이름이다.
     * @param value 기록할 숫자다. NaN과 Infinity는 JSON에서 null로 저장한다.
     * @param unit value에 적용되는 단위 문자열이다.
     */
    void RecordMetric(const std::string& name, double value, const std::string& unit) noexcept;

    /** @brief 한 번의 작업이 걸린 시간을 나노초 단위로 대기열에 추가한다. */

    /** @brief 대기 중인 기록을 파일에 반영할 때까지 기다린다. */
    void Flush() noexcept;

    /** @brief 새 기록을 중단하고 남은 기록을 저장한 뒤 소비자 스레드를 종료한다. */
    void Shutdown() noexcept;

    /** @brief 제한된 대기열에서 공간이 없어 버린 기록의 누적 개수를 반환한다. */
    [[nodiscard]] std::uint64_t GetDroppedRecordCount() const noexcept;

    /** @brief 파일을 열거나 기록하고 반영하는 중 확인한 오류의 누적 개수를 반환한다. */
    [[nodiscard]] std::uint64_t GetWriteFailureCount() const noexcept;

private:
    enum class RecordKind
    {
        Log,
        Metric
    };

    struct Record
    {
        RecordKind kind = RecordKind::Log;
        LogLevel level = LogLevel::Info;
        std::int64_t timestampMilliseconds = 0;
        std::uint64_t threadId = 0;
        std::string name;
        std::string message;
        std::string unit;
        double value = 0.0;
    };

    void Enqueue(Record record) noexcept;
    void Consume();
    void WriteRecord(const Record& record);
    static void WriteEscapedJson(std::ostream& output, const std::string& value);

    LoggerOptions m_Options;
    std::mutex m_LifecycleMutex;
    mutable std::mutex m_Mutex;
    std::condition_variable m_QueueChanged;
    std::condition_variable m_Drained;
    std::deque<Record> m_Queue;
    std::ofstream m_Output;
    std::thread m_Worker;
    std::atomic<std::uint64_t> m_DroppedRecords{0};
    std::atomic<std::uint64_t> m_WriteFailures{0};
    bool m_Accepting = true;
    bool m_Writing = false;
};
} // namespace grasplink::diagnostics
