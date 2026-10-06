#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

/**
 * @brief 기대한 조건이 만족되지 않으면 예외를 던져 현재 테스트를 실패시킨다.
 * @param condition 참이면 테스트를 계속하고 거짓이면 실패 처리할 논리값.
 * @param message 조건이 거짓일 때 실패 원인과 함께 기록할 설명.
 * @throws std::runtime_error condition이 false인 경우 message를 담아 던진다.
 */
inline void Require(bool condition, const std::string& message)
{
    if (!condition) throw std::runtime_error(message);
}

/**
 * @brief 측정값이 유한하고 기대값과의 차이가 정한 범위 안에 있는지 확인한다.
 * @param actual 시험 코드가 실제로 얻은 측정값.
 * @param expected 측정값과 같은 단위를 사용하는 기대값.
 * @param tolerance 허용하는 최대 절대 차이. actual과 expected와 단위가 같다.
 * @throws std::runtime_error actual이 유한한 수가 아니거나 두 값의 차이가 tolerance를 넘는 경우.
 */
inline void RequireNear(double actual, double expected, double tolerance, const std::string& message)
{
    Require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
}

/**
 * @brief 전달한 동작을 실행해 지정한 종류의 예외가 발생하는지 확인한다.
 * @tparam Exception 기대하는 예외 형식.
 * @tparam Function 검사할 호출 가능 객체 형식.
 * @param function 지정한 예외를 발생시켜야 하는 함수 또는 호출 객체.
 * @param message 기대한 예외가 없을 때 테스트 실패 원인으로 사용할 설명.
 * @throws std::runtime_error 지정한 예외가 발생하지 않았을 때 던진다.
 */
template<typename Exception, typename Function>
void ExpectThrows(Function&& function, const std::string& message)
{
    try { function(); }
    catch (const Exception&) { return; }
    throw std::runtime_error(message);
}
