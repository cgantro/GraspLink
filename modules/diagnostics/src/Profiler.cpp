#include "diagnostics/Profiler.h"

#include <utility>

namespace grasplink::diagnostics
{
Profiler::ProfileScope::ProfileScope(Logger& logger, std::string name)
    : m_Logger(&logger), m_Name(std::move(name)), m_Start(std::chrono::steady_clock::now()),
      m_UncaughtExceptions(std::uncaught_exceptions())
{
}

Profiler::ProfileScope::~ProfileScope() noexcept
{
    if (!m_Logger)
        return;

    const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - m_Start).count();
    m_Logger->RecordProfile(m_Name, duration, std::uncaught_exceptions() > m_UncaughtExceptions);
}

Profiler::ProfileScope::ProfileScope(ProfileScope&& other) noexcept
    : m_Logger(std::exchange(other.m_Logger, nullptr)), m_Name(std::move(other.m_Name)),
      m_Start(other.m_Start), m_UncaughtExceptions(other.m_UncaughtExceptions)
{
}

Profiler::Profiler(Logger& logger) noexcept
    : m_Logger(logger)
{
}

Profiler::ProfileScope Profiler::Measure(std::string name) const
{
    return ProfileScope(m_Logger, std::move(name));
}

void Profiler::Metric(const std::string& name, double value, const std::string& unit) const noexcept
{
    m_Logger.RecordMetric(name, value, unit);
}
} // namespace grasplink::diagnostics
