#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

/** @brief 조건이 거짓이면 테스트를 실패시킨다.
 * @param condition 검사할 조건.
 * @param message 실패 원인을 설명할 메시지.
 * @throws std::runtime_error condition이 거짓인 경우.
 */
inline void Require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}

/**
 * @brief 유한한 실제값이 기대값 허용오차 안에 있는지 확인한다.
 * @param actual 측정값.
 * @param expected 기준값과 같은 단위의 기대값.
 * @param tolerance 허용 절대 오차. 단위는 actual/expected와 같다.
 * @throws std::runtime_error actual이 유한하지 않거나 절대 차이가 tolerance보다 큰 경우.
 */
inline void RequireNear(double actual, double expected, double tolerance, const std::string& message)
{
    Require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
}

/** @brief 실행 중 지정한 예외 형식이 발생하는지 확인한다.
 * @tparam Exception 기대하는 예외 형식.
 * @tparam Function 검사할 호출 가능 객체 형식.
 * @param function 예외가 발생해야 하는 동작.
 * @param message 예외가 없으면 보고할 실패 메시지.
 * @throws std::runtime_error 지정한 예외가 발생하지 않은 경우.
 */
template<typename Exception, typename Function>
void ExpectThrows(Function&& function, const std::string& message)
{
    try { function(); }
    catch (const Exception&) { return; }
    throw std::runtime_error(message);
}
