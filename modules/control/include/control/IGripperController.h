#pragma once

#include "control/ControlTypes.h"

namespace control
{

/**
 * @brief Robotiq 2F-85 Hardware와 Simulation이 공통으로 구현할 그리퍼 제어 인터페이스.
 *
 * @details
 * 상위 계층은 Modbus RTU register packing이나 GLB finger joint를 직접 다루지 않는다.
 * Robotiq의 공개 제어 의미(rPR/rSP/rFR, gPO/gOBJ 등)는 공통 contract에 보존한다.
 */
class IGripperController
{
public:
    virtual ~IGripperController() = default;

    /** @brief Backend 연결 또는 Simulation 초기화. */
    virtual Result Connect() = 0;

    virtual void Disconnect() noexcept = 0;

    [[nodiscard]] virtual bool IsConnected() const noexcept = 0;

    /**
     * @brief 그리퍼 초기화/activation을 요청한다.
     * 실제 2F-85 backend에서는 rACT sequence에 대응한다.
     */
    virtual Result Activate() = 0;

    /** @brief 그리퍼 reset 상태로 전환한다. */
    virtual Result Reset() = 0;

    /**
     * @brief Position/Speed/Force 요청을 한 번에 설정하고 Go-To 동작을 요청한다.
     * 실제 2F-85 backend에서는 rPR/rSP/rFR + rGTO에 대응한다.
     */
    virtual Result Command(const GripperCommand& command) = 0;

    /** @brief 진행 중인 gripper motion을 정지한다. */
    virtual Result Stop() = 0;

    [[nodiscard]] virtual GripperState GetState() const = 0;

    /**
     * @brief 한 제어 주기 갱신.
     * Hardware는 status register polling, Simulation은 free-space/contact state 갱신에 사용한다.
     */
    virtual void Update(double dtSeconds) = 0;
};

} // namespace control
