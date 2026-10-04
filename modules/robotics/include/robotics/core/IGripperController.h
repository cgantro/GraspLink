#pragma once

#include "robotics/core/ControlTypes.h"

namespace grasplink::robotics
{

/**
 * @brief Hardware/Simulation gripper backend가 공통으로 구현할 인터페이스.
 *
 * 특정 register packing이나 GLB finger joint를 상위 계층에 노출하지 않는다.
 */
class IGripperController
{
public:
    virtual ~IGripperController() = default;

    /*
     * [추가 용어 설명]
     * - Gripper: 로봇 끝단에서 물체를 잡는 집게형 말단장치.
     * - Activate: 장치를 제어 가능한 준비 상태로 만드는 초기화 절차.
     * - Reset: fault/activation state를 초기 상태로 되돌리는 절차.
     * - Raw request: mm, N, rad 같은 물리량이 아니라 장치 protocol이 받는 정수 명령값.
     * - Feedback: 장치가 실제로 어떤 상태인지 다시 읽어온 값.
     * - Backend: 실제 Modbus/Serial 장치 또는 Simulation을 구현하는 내부 계층.
     */

    /** @brief backend를 사용할 수 있도록 연결/초기화한다. */
    virtual Result Connect() = 0;

    /** @brief 통신/Simulation 리소스를 정리하고 연결 해제 상태로 만든다. */
    virtual void Disconnect() noexcept = 0;

    /** @return command를 처리할 준비가 되어 있으면 true. */
    [[nodiscard]] virtual bool IsConnected() const noexcept = 0;

    /**
     * @brief 장치 activation 절차를 시작/수행한다.
     * @note Robotiq 2F-85 hardware에서는 rACT 계열 절차에 대응한다.
     */
    virtual Result Activate() = 0;

    /** @brief 장치 또는 Simulation의 gripper state를 초기 상태로 되돌린다. */
    virtual Result Reset() = 0;

    /**
     * @brief 위치/속도/힘 요청을 전달한다.
     * @param command 0..255 raw request. position 값은 mm나 rad 자체가 아니다.
     */
    virtual Result Command(const GripperCommand& command) = 0;

    /** @brief 현재 gripper motion을 software 수준에서 정지한다. */
    virtual Result Stop() = 0;

    /** @return activation/contact/fault/position/current 등을 담은 상태 snapshot. */
    [[nodiscard]] virtual GripperState GetState() const = 0;

    /**
     * @brief backend 내부 상태를 한 제어 step 갱신한다.
     * @param dtSeconds 경과 시간 [s].
     */
    virtual void Update(double dtSeconds) = 0;
};

} // namespace grasplink::robotics
