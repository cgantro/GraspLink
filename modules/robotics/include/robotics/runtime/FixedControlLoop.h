#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <type_traits>


namespace grasplink::robotics::runtime
{
/**
 * @brief 화면 frame 사이의 시간을 모아 같은 길이의 제어·물리 tick을 실행한다.
 * @details Frame은 화면을 한 번 그리는 주기이며 그 사이의 시간은 일정하지 않을 수 있다.
 * 이 loop는 그 시간을 accumulator에 모으고 fixedDeltaSeconds가 쌓일 때마다 tick 함수를 호출한다.
 * 짧은 frame은 남은 시간을 보관하고 긴 frame은 여러 tick을 실행할 수 있다.
 * maxFrameDeltaSeconds보다 긴 지연은 잘라 버리므로 긴 멈춤 뒤 과거 tick을 무한히 따라잡지 않는다.
 * 버린 시간만큼 Simulation은 실제 시계보다 뒤처질 수 있다.
 * 이 클래스는 OS thread를 예약하지 않으며 실시간 응답도 보장하지 않는다.
 * Viewer는 250 Hz, 즉 0.004 s tick과 0.1 s frame 입력 상한을 사용한다.
 */
class FixedControlLoop final{
public:
    /**
     * @brief 한 번의 제어 tick 길이와 frame 하나에서 받아들일 최대 시간을 정한다.
     * @param fixedDeltaSeconds tick 간격 [s]. 유한한 양수여야 한다.
     * @param maxFrameDeltaSeconds frame마다 누적할 경과시간 상한 [s]. 유한한 양수여야 한다.
     * @throws std::invalid_argument 두 값 중 하나가 유한한 양수가 아니면 발생한다.
     */
    explicit FixedControlLoop(double fixedDeltaSeconds, double maxFrameDeltaSeconds = 0.1):
            fixedDeltaSeconds_(fixedDeltaSeconds), maxFrameDeltaSeconds_(maxFrameDeltaSeconds){
        if(!std::isfinite(fixedDeltaSeconds_) || fixedDeltaSeconds <= 0.0) 
            throw std::invalid_argument("FixedControlLoop: fixedDeltaSeconds must be finite and > 0.");

        if (!std::isfinite(maxFrameDeltaSeconds_) || maxFrameDeltaSeconds_ <= 0.0)
            throw std::invalid_argument("FixedControlLoop: maxFrameDeltaSeconds must be finite and > 0.");
    }
        /**
     * @brief 지난 frame 시간을 누적해 준비된 고정 간격만큼 실행한다.
     * @param frameDeltaSeconds 직전 화면 frame 이후 경과 시간 [s]다. 유한한 양수만 처리한다.
     * @param tick fixedDeltaSeconds를 받아 Controller나 Simulation을 한 번 진행하는 함수다.
     * @return 이번 호출에서 실행한 tick 횟수다.
     * @details 유효하지 않은 frame 시간은 무시하고 0을 반환한다.
     * 한 frame 입력은 maxFrameDeltaSeconds까지만 누적하며 남은 시간은 다음 호출에 보관한다.
     * tick이 예외를 던지면 호출자에게 전달하고 해당 tick의 시간은 accumulator에서 빼지 않는다.
     */
    template<typename TickFunction>
    std::size_t Advance(double frameDeltaSeconds, TickFunction&& tick){
        static_assert(std::is_invocable_v<TickFunction&, double>,"TickFunction must be callable with double fixedDeltaSeconds.");

        // 음수, 0, NaN, Inf는 정상적인 시간이 아니므로 simulation clock에 넣지 않는다.
        if(!std::isfinite(frameDeltaSeconds) || frameDeltaSeconds <= 0) return 0;

        // 디버거 정지나 frame stall 뒤 뒤늦은 update가 몰리는 양을 제한한다.
        const double clampedFrameDeltaSeconds = std::min(frameDeltaSeconds,maxFrameDeltaSeconds_);

        // 한 번에 처리할 수 없는 초과 시간은 버리고 나중에 보충하지 않는다. 따라서 긴 멈춤 뒤에는 실제 시계(wall-clock)보다 시뮬레이션 시간이 적게 흐를 수 있다.
        accumulatorSeconds_ += clampedFrameDeltaSeconds;
        std::size_t executedSteps = 0;

        // tick마다 같은 dt를 써 controller/physics 계산을 render frame 길이와 독립시킨다.
        while(accumulatorSeconds_ >= fixedDeltaSeconds_){
            tick(fixedDeltaSeconds_);
            accumulatorSeconds_ -= fixedDeltaSeconds_;
            ++executedSteps;
        }

        return executedSteps;
    }

    /** @brief 잔여 시간을 버린다. Reset이나 재연결처럼 이전 simulation 시간축을 끊을 때 사용한다. */
    void Reset() noexcept { accumulatorSeconds_ = 0.0;} 

    /** @brief 한 tick의 고정 시간 간격 [s]. */
    [[nodiscard]]
    double GetFixedDeltaSeconds() const noexcept{return fixedDeltaSeconds_;}

    /** @brief 다음 tick 전까지 누적된 잔여 시간 [s]. Advance 후에는 보통 0 이상 fixedDelta 미만이다. */
    [[nodiscard]]
    double GetAccumulatorSeconds() const noexcept{return accumulatorSeconds_;}

    /** @brief accumulator/fixedDeltaSeconds. 렌더링에서 이전·현재 상태를 보간할 때 쓰는 alpha [0,1). */
    [[nodiscard]]
    double GetInterpolationAlpha() const noexcept
    {
        return accumulatorSeconds_ / fixedDeltaSeconds_;
    }
private:
    // Controller와 Simulation이 한 tick 동안 상태를 전진시키는 고정 시간 간격 [s]다.
    double fixedDeltaSeconds_;

    // 렌더링 frame 하나에서 simulation clock에 누적하도록 허용하는 최대 경과시간 [s]다.
    double maxFrameDeltaSeconds_;

    // Fixed Update를 한 번 더 실행하기에는 부족해 누적기에 남아 있는 시간 [s]다.
    double accumulatorSeconds_ = 0.0;
};
} // namespace 
