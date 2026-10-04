#pragma once

#include "control/ControlTypes.h"

namespace control
{

/**
 * @brief Hardware Robot Controller와 Simulation Controller가 공통으로 구현할 로봇 제어 인터페이스.
 *
 * @details
 * 상위 계층은 제조사 protocol, Flecs Entity, GLB hierarchy를 직접 알지 않는다.
 * Joint 개수/이름/축/limit은 model별 RobotSpecification이 결정한다.
 */
class IRobotController
{
public:
    virtual ~IRobotController() = default;

    virtual Result Connect() = 0;
    virtual void Disconnect() noexcept = 0;
    [[nodiscard]] virtual bool IsConnected() const noexcept = 0;

    /**
     * @brief 관절 공간 절대 목표로 이동을 요청한다.
     * @note targetPositionRadians의 원소 개수와 순서는 현재 backend의 RobotSpecification과 일치해야 한다.
     */
    virtual Result MoveJoint(const JointMoveCommand& command) = 0;

    /** @brief TCP를 Cartesian 직선 경로로 이동하도록 요청한다. */
    virtual Result MoveLinear(const LinearMoveCommand& command) = 0;

    /**
     * @brief 현재 진행 중인 동작을 정지시킨다.
     * @warning 실제 비상정지(E-Stop) 회로와 동일한 기능을 의미하지 않는다.
     */
    virtual Result Stop() = 0;

    [[nodiscard]] virtual RobotState GetState() const = 0;

    /**
     * @brief Controller backend를 한 주기 갱신한다.
     * Simulation은 target 추종/FSM, Hardware는 feedback polling에 사용한다.
     */
    virtual void Update(double dtSeconds) = 0;
};

} // namespace control
