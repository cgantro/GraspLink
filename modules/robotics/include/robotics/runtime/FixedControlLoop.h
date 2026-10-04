#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <type_traits>


namespace grasplink::robotics::runtime
{
/**
 * @brief 가변적인 frame delta time을 고정된 timestep으로 나누어 실행한다.
 *
 * Rendering FPS와 Robot/Simulation update 주기를 분리하기 위한 runtime utility다.
 *
 * 예:
 *
 * fixedDeltaSeconds = 0.004 s
 * frameDeltaSeconds = 0.010 s
 *
 * accumulator = 0.010
 *
 * tick(0.004)
 * accumulator = 0.006
 *
 * tick(0.004)
 * accumulator = 0.002
 *
 * 남은 0.002초는 다음 rendering frame으로 이월한다.
 *
 * 따라서 rendering frame 시간이 매번 달라져도 tick callback은 항상 동일한
 * fixedDeltaSeconds를 전달받는다.
 *
 * @note
 * 이 클래스는 hard real-time scheduler가 아니다.
 * 별도 thread를 생성하거나 sleep을 호출하지 않는다.
 */

class FixedControlLoop final{
public:
    /**
    * @param fixedDeltaSeconds
    *        하나의 fixed tick이 나타내는 시간 [s].
    *
    *        예:
    *        250 Hz -> 1 / 250 = 0.004 s
    *
    * @param maxFrameDeltaSeconds
    *        한 번의 Advance()에서 받아들일 최대 frame 경과시간 [s].
    *        breakpoint, debugger 정지, window drag 등의 이유로
    *        frameDeltaSeconds가 비정상적으로 크게 들어왔을 때
    *        수백~수천 번의 fixed update가 한 frame에서 몰아서 실행되는 것을 방지한다.
    */
    explicit FixedControlLoop(double fixedDeltaSeconds, double maxFrameDeltaSeconds = 0.1):
            fixedDeltaSeconds_(fixedDeltaSeconds), maxFrameDeltaSeconds_(maxFrameDeltaSeconds){
        if(!std::isfinite(fixedDeltaSeconds_) || fixedDeltaSeconds <= 0.0) 
            throw std::invalid_argument("FixedControlLoop: fixedDeltaSeconds must be finite and > 0.");

        if (!std::isfinite(maxFrameDeltaSeconds_) || maxFrameDeltaSeconds_ <= 0.0)
            throw std::invalid_argument("FixedControlLoop: maxFrameDeltaSeconds must be finite and > 0.");
    }
    /**
     * @brief 한 rendering frame의 경과시간을 누적하고 필요한 fixed tick을 실행한다.
     *
     * @param frameDeltaSeconds
     *        이전 rendering frame부터 현재 frame까지 실제 경과한 시간 [s].
     *
     * @param tick
     *        fixed timestep마다 호출할 callback.
     *
     *        다음 형태로 호출 가능해야 한다.
     *
     *        void tick(double fixedDeltaSeconds);
     *
     * @return 이번 Advance()에서 실행한 fixed tick 횟수.
    */
    template<typename TickFunction>
    std::size_t Advance(double frameDeltaSeconds, TickFunction&& tick){
        static_assert(std::is_invocable_v<TickFunction&, double>,"TickFunction must be callable with double fixedDeltaSeconds.");

        /*
            음수, 0, NaN, Inf는 정상적인 시간이 아님
        */
        if(!std::isfinite(frameDeltaSeconds) || frameDeltaSeconds <= 0) return 0;

        // 3. 시간 폭주 방지 (Spike Clamping): 디버깅 등의 이유로 너무 큰 값이 들어오면 상한선으로 제한
        const double clampedFrameDeltaSeconds = std::min(frameDeltaSeconds,maxFrameDeltaSeconds_);

        // 4. 입력받은 시간을 누적기에 저장
        accumulatorSeconds_ += clampedFrameDeltaSeconds;
        std::size_t executedSteps = 0;

        //  5. 고정된 시간(fixedDeltaSeconds_)만큼 누적된 시간이 쌓여있다면, 그 횟수만큼 반복 호출
        while(accumulatorSeconds_ >= fixedDeltaSeconds_){
            tick(fixedDeltaSeconds_);
            accumulatorSeconds_ -= fixedDeltaSeconds_;
            ++executedSteps;
        }

        // 6. 이번 프레임에서 고정 타임스텝이 총 몇 번 실행되었는지 반환
        return executedSteps;
    }

    /**
     * @brief 아직 처리되지 않은 누적 시간을 모두 제거한다.
     *
     * 다음과 같은 상황에서 사용할 수 있다.
     *
     * - Simulation Reset
     * - Controller Reconnect
     * - Scene Reset
     *
     * 예를 들어 accumulator에 0.003초가 남아 있는 상태에서
     * Simulation을 처음부터 다시 시작한다면,
     * 이전 Simulation의 남은 시간을 새 Simulation에 적용하면 안 되므로 Reset한다.
    */
    void Reset() noexcept { accumulatorSeconds_ = 0.0;} 

    /**
     * @brief 한 번의 fixed update가 사용하는 고정 시간 간격을 반환한다.
     *
     * @return fixed timestep [s].
    */
    [[nodiscard]]
    double GetFixedDeltaSeconds() const noexcept{return fixedDeltaSeconds_;}

    /**
     * @brief 아직 simulation/control에 반영되지 않은 누적 시간을 반환한다.
     *
     * 항상 정상적인 Advance() 실행이 끝난 뒤에는 일반적으로:
     *     0 <= accumulator < fixedDelta
     * 관계를 가진다.
     *
     * @return 남아 있는 누적 시간 [s].
    */
    [[nodiscard]]
    double GetAccumulatorSeconds() const noexcept{return accumulatorSeconds_;}

    /**
     * @brief 다음 물리 계산까지 시간이 얼마나 애매하게 남았는지 '비율(0.0 ~ 1.0)'로 알려줍니다.
     *
     * 물리 연산은 고정된 시간(예: 0.004초)마다 뚝뚝 끊어져서 계산됩니다.
     * 하지만 화면(렌더링)은 훨씬 더 자주 그려집니다.
     * 
     * 만약 다음 물리 계산까지 40% 정도 시간이 흘렀다면, 이 함수는 0.4를 반환합니다.
     * 이 값(alpha)을 사용하면 화면을 그릴 때 '이전 위치'와 '다음 위치' 사이를 
     * 부드럽게 이어 붙여서(보간) 로봇이 순간이동 하지 않고 부드럽게 움직이게 만들 수 있습니다.
     *
     * 예시:
     *     fixedDt     = 0.004초 (물리 계산 주기)
     *     accumulator = 0.001초 (남은 짜투리 시간)
     *     alpha       = 0.001 / 0.004 = 0.25 (다음 단계까지 25%만큼 가 있는 상태)
     *
     * 현재 RobotTransformAdapter에서는 굳이 안 써도 되지만,
     * 나중에 화면 렌더링이 툭툭 끊겨 보일 때 부드럽게 만드는 용도로 사용합니다.
     *
     * @return 다음 물리 단계까지의 진행률 (0.0 <= alpha < 1.0)
    */
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
