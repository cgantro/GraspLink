#pragma once

#include <array>
#include <cstdint>

namespace grasplink::simulation::scenario
{

/**
 * @brief 무작위 pick-and-place 표본을 지정한 seed에서 다시 생성하도록 초기화한다.
 * @param seed 같은 seed와 같은 함수 호출 순서를 사용하면 같은 좌표와 회전 순서를 만든다.
 * @details 표본기는 상자와 목표 위치의 직전 좌표도 기억해 연속 임무가 서로 너무 가까워지지 않게 한다. 이 함수를 부르면 난수 상태와 두 위치 기록이 함께 초기화된다.
 */
void SeedRandom(std::uint32_t seed);

/** @brief 로봇 작업 반경 안에서 다음 상자 중심 위치 [m]를 고른다. */
[[nodiscard]] std::array<float, 3> SampleBoxPosition();

/** @brief 로봇 작업 반경 안에서 다음 배치 영역 중심 위치 [m]를 고른다. */
[[nodiscard]] std::array<float, 3> SamplePlacementPosition();

/** @brief 상자나 배치 영역에 사용할 Y축 회전각 [rad]을 고른다. */
[[nodiscard]] float SamplePlanarRotation();

} // namespace grasplink::simulation::scenario
