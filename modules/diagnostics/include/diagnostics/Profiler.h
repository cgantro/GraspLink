#pragma once

#include "diagnostics/Logger.h"

#include <chrono>
#include <exception>
#include <string>
#include <utility>

namespace grasplink::diagnostics
{
/**
 * @brief 이름 있는 작업의 실행 시간과 예외를 Logger에 전달한다.
 * @details Trace는 함수를 감싸는 데코레이터처럼 사용할 수 있어 호출 경계에 계측을 붙인다.
 * ProfileScope는 블록 범위의 시간을 측정하며 소멸할 때 결과를 비동기 대기열에 전달한다.
 * Profiler와 ProfileScope는 Logger를 빌려 쓰므로 Logger가 Profiler와 모든 Scope보다 오래 살아 있어야 한다.
 */
class Profiler final
{
public:
    /**
     * @brief 이름이 붙은 범위의 시작과 끝을 이용해 실행 시간을 기록한다.
     * @details Logger를 빌려 쓰므로 이 Scope가 살아 있는 동안 Logger가 파괴되어서는 안 된다.
     */
    class ProfileScope final
    {
    public:
        ProfileScope(Logger& logger, std::string name);
        ~ProfileScope() noexcept;

        ProfileScope(const ProfileScope&) = delete;
        ProfileScope& operator=(const ProfileScope&) = delete;
        ProfileScope(ProfileScope&& other) noexcept;
        ProfileScope& operator=(ProfileScope&&) = delete;

    private:
        Logger* m_Logger;
        std::string m_Name;
        std::chrono::steady_clock::time_point m_Start;
        int m_UncaughtExceptions;
    };

    explicit Profiler(Logger& logger) noexcept;

    /** @brief 이름 있는 코드 범위의 실행 시간을 측정하는 Scope를 만든다. */
    [[nodiscard]] ProfileScope Measure(std::string name) const;

    /**
     * @brief callable 실행 시간을 기록하고 예외를 로그에 남긴 뒤 원래 호출자에게 전달한다.
     * @param name 실행 범위를 나타내는 이름이다.
     * @param function 계측할 작업이다. 인자 없이 한 번 호출된다.
     * @return callable이 반환한 값을 같은 형식으로 반환한다.
     * @throws callable이 던진 예외를 기록 시도 후 그대로 전달한다.
     */
    template<typename Function>
    decltype(auto) Trace(const std::string& name, Function&& function) const
    {
        auto scope = Measure(name);
        try
        {
            return std::forward<Function>(function)();
        }
        catch (const std::exception& error)
        {
            try
            {
                m_Logger.Write(LogLevel::Error, name, error.what());
            }
            catch (...)
            {
            }
            throw;
        }
        catch (...)
        {
            try
            {
                m_Logger.Write(LogLevel::Error, name, "알 수 없는 예외가 발생했습니다.");
            }
            catch (...)
            {
            }
            throw;
        }
    }

    /** @brief 단위가 명시된 수치 기록을 Logger에 전달한다. */
    void Metric(const std::string& name, double value, const std::string& unit) const noexcept;

private:
    Logger& m_Logger;
};
} // namespace grasplink::diagnostics
