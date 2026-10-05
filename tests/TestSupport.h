#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

inline void Require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}

inline void RequireNear(double actual, double expected, double tolerance, const std::string& message)
{
    Require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
}

template<typename Exception, typename Function>
void ExpectThrows(Function&& function, const std::string& message)
{
    try { function(); }
    catch (const Exception&) { return; }
    throw std::runtime_error(message);
}
