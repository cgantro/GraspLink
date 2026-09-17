#pragma once

#include <stdint.h>

namespace grasplink::controller
{
/**
 * @brief 사용자가 현재 potentiometer로 편집하고 있는 Cartesian 좌표 축.
 *
 * 이 값은 로봇 joint 축이 아니다. ESP32는 base/world 좌표계의 목표점 `(x, y, z)`만
 * 만든다. TCP orientation 보완, IK, joint limit 검증은 simulator의 책임이다.
 */
enum class TargetAxis : uint8_t
{
    X = 0,
    Y = 1,
    Z = 2,
};

/**
 * @brief 하나의 Cartesian 축에서 허용하는 목표값 구간 [minimum, maximum].
 *
 * 단위는 meter(m)이다. ADC의 무차원 raw 값 `r`은 먼저 [0, adcMaximum]으로
 * clamp되고, 아래 선형식으로 이 범위에 대응된다.
 *
 * `value = minimum + (maximum - minimum) * r / adcMaximum`
 *
 * 이 mapping을 controller에 두는 이유는 입력 장치가 표시하는 값과 UDP로 보내는
 * 값이 항상 같게 하기 위함이다. 로봇의 실제 reachability는 이 workspace 안에서도
 * 보장하지 않으며 simulator IK가 별도로 판단한다.
 */
struct CoordinateRange
{
    float minimum = 0.0F;
    float maximum = 0.0F;
};

/**
 * @brief 보드와 무관한 target 입력 정책.
 *
 * `adcMaximum`은 ADC 분해능이 아니라 드라이버가 controller에 넘기는 유효 최대
 * code이다. 예를 들어 12-bit raw ADC면 보통 4095이다. `targetId`는 하나의
 * controller가 여러 scene object를 제어할 수 있도록 protocol sender에 전달된다.
 */
struct TargetInputConfiguration
{
    // Host `.env.example`와 같은 v1 default workspace다. 실제 board build에서는
    // generated configuration/DeviceTree-adjacent setting으로 이 값을 함께 바꾼다.
    CoordinateRange x{-0.90F, 0.90F};
    CoordinateRange y{0.10F, 1.20F};
    CoordinateRange z{-0.90F, 0.90F};
    uint16_t adcMaximum = 4095U;
    uint32_t longPressThresholdMs = 700U;
    uint32_t targetId = 1U;
};

/**
 * @brief UDP transport에 넘길, orientation을 포함하지 않는 사용자 target snapshot.
 *
 * 좌표는 simulator world/base frame 기준 meter(m)이다. 이 type은 wire packet이
 * 아니다. transport 계층이 이 type과 송신 sequence를 받아 v1 TargetCommand
 * packet으로 직렬화한다. 따라서 버튼을 누르는 순간의 세 축이 원자적으로 함께
 * 전송되며, 이후 ADC 변화가 이미 요청된 snapshot을 바꾸지 않는다.
 */
struct TargetSnapshot
{
    uint32_t targetId = 1U;
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

/**
 * @brief button release를 application context에서 해석한 결과.
 *
 * ISR은 이 함수를 호출하지 않는다. ISR은 edge와 tick만 bounded queue에 넣고,
 * worker가 press duration을 계산하여 이 API로 전달한다. 이렇게 하면 GPIO ISR에
 * ADC/I2C/socket 작업이 들어가지 않는다.
 */
enum class ButtonAction : uint8_t
{
    AxisChanged,
    SendSnapshot,
};

/**
 * @brief potentiometer 하나와 button 하나로 XYZ target을 편집하는 무할당 상태기계.
 *
 * - short press (< longPressThresholdMs): `X -> Y -> Z -> X` 선택 축 전환
 * - long press (>= longPressThresholdMs): 현재 XYZ를 전송할 snapshot으로 확정
 *
 * 이 class는 thread-safe하지 않다. ADC sample과 button action은 같은 Zephyr work
 * queue/thread에서 순서대로 호출해야 한다. 그러면 snapshot은 별도 lock 없이도
 * 일관된 세 축 값을 가진다. ISR과 UI/UDP thread가 직접 동시에 접근해야 한다면
 * application이 queue로 직렬화해야 하며, 이 class에 mutex를 추가하지 않는다.
 */
class TargetInputController final
{
public:
    explicit TargetInputController(const TargetInputConfiguration& configuration) noexcept;

    /** @brief 현재 선택된 축에 raw ADC 값을 반영한다. ADC thread/work context 전용. */
    void OnAdcSample(uint16_t rawCode) noexcept;

    /**
     * @brief release된 button의 hold 시간으로 short/long press를 결정한다.
     * @return long press면 SendSnapshot, 그렇지 않으면 축을 다음 축으로 바꾸고 AxisChanged.
     */
    ButtonAction OnButtonReleased(uint32_t heldMs) noexcept;

    /** @brief 현재 편집 상태를 복사한다. 반환값은 dispatch할 때 그대로 보관한다. */
    TargetSnapshot Snapshot() const noexcept;

    /** @brief OLED 등 UI가 표시할 현재 편집 축. */
    TargetAxis SelectedAxis() const noexcept;

private:
    static float MapAdcToRange(uint16_t rawCode, uint16_t adcMaximum,
                               const CoordinateRange& range) noexcept;
    static TargetAxis NextAxis(TargetAxis axis) noexcept;

    TargetInputConfiguration configuration_{};
    TargetAxis selectedAxis_ = TargetAxis::X;
    TargetSnapshot snapshot_{};
};
} // namespace grasplink::controller
