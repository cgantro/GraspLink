#pragma once

#include <cstddef>

namespace grasplink::robotics::backends::simulation::detail
{
/**
 * @brief 시뮬레이션 경로의 충돌 검사와 실시간 추종에 쓰는 기본 정책값이다.
 * @details 간격을 좁히면 검사 정확도가 높아지지만 IK·충돌 검사 횟수와 저장 경로가 늘어난다.
 * 재시도 한도와 진행률 감소율은 한 제어 주기의 계산량과 속도 제한 준수를 함께 조절한다.
 */
struct SimulationMotionPolicy final
{
    // 관절 경로 충돌 검사를 이 각도 간격으로 나눠 중간 접촉을 놓칠 가능성과 검사량을 절충한다.
    double jointCollisionSampleSpacingRadians = 0.08;
    // 직선 TCP 경로의 위치 오차가 이 간격보다 커지지 않도록 IK 표본을 둔다.
    double linearPositionSampleSpacingMeters = 0.01;
    // 회전 구간도 표본 사이의 방향 변화가 커지지 않도록 별도 간격으로 나눈다.
    double linearOrientationSampleSpacingRadians = 0.05;
    // 한 명령의 계획 시간과 저장할 IK 표본 수가 비정상적으로 커지지 않게 제한한다.
    std::size_t maximumPathIntervals = 4096;
    // 속도 상한을 만족하는 진행률을 찾되 한 제어 주기의 재검사 횟수를 제한한다.
    std::size_t linearRuntimeRetryAttempts = 16;
    // 특이 자세에서 TCP를 유지할 작은 관절 보조 이동을 단계적으로 줄여 찾는다.
    std::size_t reorientationRetryAttempts = 12;
    // 재시도마다 추정 진행률을 80%로 줄여 속도 상한을 넘지 않을 여유를 둔다.
    double runtimeRetryProgressScale = 0.8;
};

inline constexpr SimulationMotionPolicy kDefaultSimulationMotionPolicy{};
}
