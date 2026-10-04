#pragma once

#include "control/ControlTypes.h"

namespace control
{

/**
 * @brief 실제 HCR Controller와 Simulation Controller가 공통으로 구현할 로봇팔 제어 인터페이스.
 *
 * @details
 * 상위 계층은 Hanwha TCP/IP/Modbus TCP, Flecs Entity, GLB hierarchy를 직접 알지 않는다.
 * 실제 장비와 시뮬레이터 모두 이 인터페이스 뒤에 둔다.
 *
 * MoveJoint/MoveLinear는 고수준 motion command다. 특정 제조사의 script 함수명이나
 * packet format을 이 인터페이스에 노출하지 않는다.
 */
class IRobotController
{
public:
    virtual ~IRobotController() = default;

    /** @brief Backend와 연결하거나 Simulation backend를 초기화한다. */
    virtual Result Connect() = 0;

    /** @brief 연결/리소스를 정리한다. 예외를 던지지 않는다. */
    virtual void Disconnect() noexcept = 0;

    [[nodiscard]] virtual bool IsConnected() const noexcept = 0;

    /**
     * @brief 관절 공간 목표로 이동을 요청한다.
     * @note targetPositionRadians는 J1~J6 순서의 절대 관절각이다.
     */
    virtual Result MoveJoint(const JointMoveCommand& command) = 0;

    /** @brief TCP를 Cartesian 직선 경로로 이동하도록 요청한다. */
    virtual Result MoveLinear(const LinearMoveCommand& command) = 0;

    /**
     * @brief 현재 진행 중인 동작을 정지시킨다.
     * @warning 실제 비상정지(E-Stop) 회로와 동일한 기능을 의미하지 않는다.
     */
    virtual Result Stop() = 0;

    /** @brief 현재 backend에서 관측한 Robot 상태 snapshot을 반환한다. */
    [[nodiscard]] virtual RobotState GetState() const = 0;

    /**
     * @brief Controller backend를 한 주기 갱신한다.
     *
     * Simulation에서는 target 추종/FSM을 진행하고 Hardware에서는 통신 상태와
     * feedback을 poll/갱신하는 용도로 사용한다.
     */
    virtual void Update(double dtSeconds) = 0;
};

} // namespace control
