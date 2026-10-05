#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <type_traits>


namespace grasplink::robotics::runtime
{
/**
 * @brief 렌더 frame 경과시간을 고정 간격 제어 tick으로 바꾸는 accumulator.
 * @details 매 Advance는 frame 시간을 최대 maxFrameDeltaSeconds까지만 받아 누적하고,
 * 충분한 시간이 모일 때마다 같은 fixedDeltaSeconds로 callback을 실행한다. 짧은 frame은
 * tick을 생략하고 잔여 시간을 누적하며, 긴 frame은 여러 tick을 연달아 실행할 수 있다.
 * 상한을 넘긴 시간은 버려 긴 멈춤 뒤 catch-up 폭주를 제한한다. 따라서 simulation 시간은 실제
 * 벽시계와 어긋날 수 있고 이 클래스는 thread scheduler나 hard real-time 보장을 제공하지 않는다.
 * Viewer는 250 Hz 제어 tick(0.004 s)과 최대 0.1 s frame 입력을 사용한다.
 */
class FixedControlLoop final{
public:
    /**
     * @brief 고정 tick 간격과 frame 시간 상한을 설정한다.
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
     * @brief frame 경과시간을 누적하고 준비된 고정 tick을 차례로 실행한다.
     * @param frameDeltaSeconds 직전 frame 이후 경과 시간 [s]. 유한한 양수만 사용한다.
     * @param tick 한 tick을 진행하는 callable. 각 호출에는 fixedDeltaSeconds가 전달된다.
     * @return 이번 호출에서 실행한 tick 수.
     * @details 잘못된 frame 시간은 무시하고 0을 반환한다. 큰 시간은 상한으로 자른 뒤 나머지를 버린다.
     * 상한 안에서 tick보다 작은 잔여 시간은 다음 Advance까지 보존된다. callback이 예외를 던지면 예외는
     * 호출자에게 전달되며, 해당 tick의 시간은 accumulator에서 차감되지 않는다.
     */
    template<typename TickFunction>
    std::size_t Advance(double frameDeltaSeconds, TickFunction&& tick){
        static_assert(std::is_invocable_v<TickFunction&, double>,"TickFunction must be callable with double fixedDeltaSeconds.");

        // 음수, 0, NaN, Inf는 정상적인 시간이 아니므로 simulation clock에 넣지 않는다.
        if(!std::isfinite(frameDeltaSeconds) || frameDeltaSeconds <= 0) return 0;

        // 디버거 정지나 frame stall 뒤 뒤늦은 update가 몰리는 양을 제한한다.
        const double clampedFrameDeltaSeconds = std::min(frameDeltaSeconds,maxFrameDeltaSeconds_);

        // 버린 시간은 다시 보충하지 않아 장시간 정지 뒤에는 wall-clock과 simulation 시간이 달라질 수 있다.
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
    // Controller/Simulation이 한 tick에서 전진할 고정 시간 [s].
    double fixedDeltaSeconds_;

    // 한 rendering frame에서 받아들일 최대 경과시간 [s].
    double maxFrameDeltaSeconds_;

    // 아직 fixed tick으로 소비되지 않은 잔여 시간 [s].
    double accumulatorSeconds_ = 0.0;
};
} // namespace 
