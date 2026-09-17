#pragma once

#include <stdint.h>

namespace grasplink::controller
{
/**
 * @brief simulator가 controller UI에 전달하는 사용자 수준 상태.
 *
 * protocol의 raw status byte를 이 enum으로 바꾸는 adapter는 UDP client에 둔다.
 * UI는 network ABI를 알 필요가 없고, simulator 역시 OLED/LED의 세부 표현을 알지
 * 않는다. `TransportError`는 timeout, malformed state packet 같은 controller-local
 * 오류용이다.
 */
enum class RobotFeedbackStatus : uint8_t
{
    Unknown = 0,
    Idle,
    TargetReceived,
    Moving,
    GraspSuccess,
    GraspFailed,
    TransportError,
};

/** @brief 보드 driver가 해석할 LED 상태. 실제 RGB pin level은 driver에 남긴다. */
enum class LedIndication : uint8_t
{
    Off = 0,
    Waiting,
    Active,
    Success,
    Failure,
};

/**
 * @brief 보드 driver가 한 번 재생할 buzzer event.
 *
 * `Success`/`Failure`는 level이 아닌 transition event이다. 따라서 같은 simulator
 * 상태 packet이 반복되어도 presenter가 재생을 중복 요청하지 않는다.
 */
enum class BuzzerIndication : uint8_t
{
    Silent = 0,
    Success,
    Failure,
};

/**
 * @brief feedback output에 전달하는 선언적 화면 모델.
 *
 * `status` 문자열 렌더링, OLED I2C write, LED PWM, buzzer timing은 concrete board
 * driver가 담당한다. 이 구조는 host test에서 hardware 없이 state-to-UI 정책만
 * 검증할 수 있게 한다.
 */
struct FeedbackView
{
    RobotFeedbackStatus status = RobotFeedbackStatus::Unknown;
    LedIndication led = LedIndication::Off;
    BuzzerIndication buzzer = BuzzerIndication::Silent;
};

/**
 * @brief OLED/RGB LED/buzzer를 위한 board adapter 경계.
 *
 * 호출자는 하나의 work/thread context만 사용한다. 구현체는 이 호출에서 I2C/PWM을
 * 수행해도 되지만 GPIO ISR에서는 호출되어서는 안 된다. 동적 allocation과 exception
 * 을 쓰지 않는 작은 virtual interface라 ESP32 firmware의 static instance로 둘 수 있다.
 */
class IFeedbackOutput
{
public:
    virtual ~IFeedbackOutput() = default;
    virtual void Present(const FeedbackView& view) noexcept = 0;
};

/** @brief hardware가 아직 정해지지 않은 build/host test용 무동작 output. */
class NullFeedbackOutput final : public IFeedbackOutput
{
public:
    void Present(const FeedbackView&) noexcept override {}
};

/**
 * @brief robot state를 board-independent feedback 정책으로 변환하는 stateful presenter.
 *
 * 동일 status는 suppressed한다. 특히 UDP가 마지막 `GraspSuccess`를 재전송해도
 * buzzer가 계속 울리지 않는다. status가 바뀐 순간에만 `Present`가 호출된다.
 */
class FeedbackPresenter final
{
public:
    explicit FeedbackPresenter(IFeedbackOutput& output) noexcept;

    /** @return view가 바뀌어 output에 전달되었으면 true. */
    bool OnRobotStatus(RobotFeedbackStatus status) noexcept;

    /** @brief 다음 동일 status도 다시 출력하도록 deduplication 상태를 지운다. */
    void RequestRefresh() noexcept;

    FeedbackView CurrentView() const noexcept;

private:
    static FeedbackView MakeView(RobotFeedbackStatus status) noexcept;

    IFeedbackOutput& output_;
    FeedbackView current_{};
    bool hasPresented_ = false;
};
} // namespace grasplink::controller
