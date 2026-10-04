#pragma once

#include "robotics/core/ControlTypes.h"

namespace grasplink::robotics
{

/**
 * @brief 실제 Robot backend와 Simulation backend가 공통으로 구현할 제어 인터페이스.
 *
 * 제조사 protocol, Flecs Entity, GLB hierarchy를 상위 계층에 노출하지 않는다.
 * Joint 개수/이름/축/limit은 models::RobotSpecification이 결정한다.
 */
class IRobotController
{
public:
    virtual ~IRobotController() = default;

    virtual Result Connect() = 0;
    virtual void Disconnect() noexcept = 0;
    [[nodiscard]] virtual bool IsConnected() const noexcept = 0;

    virtual Result MoveJoint(const JointMoveCommand& command) = 0;
    virtual Result MoveLinear(const LinearMoveCommand& command) = 0;

    /** @warning 실제 물리 E-Stop 회로를 대체하지 않는 software motion stop이다. */
    virtual Result Stop() = 0;

    [[nodiscard]] virtual RobotState GetState() const = 0;
    virtual void Update(double dtSeconds) = 0;
};

} // namespace grasplink::robotics
