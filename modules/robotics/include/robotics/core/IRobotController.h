#pragma once

#include "robotics/core/ControlTypes.h"

namespace grasplink::robotics
{

/**
 * @brief 실제 Robot backend와 Simulation backend가 공통으로 구현하는 로봇 제어 인터페이스.
 *
 * @details
 * 상위 계층(Application/Planner/IK)은 Hanwha TCP/IP, Modbus, Flecs Entity, GLB hierarchy 같은
 * 구현 세부사항을 알지 않는다. 모든 backend는 이 인터페이스에서 같은 단위와 command/state 의미를 사용한다.
 *
 * Robot의 관절 개수, 이름, local axis, angle/velocity limit은 models::RobotSpecification이 결정한다.
 * 따라서 이 인터페이스 자체는 6축 HCR-12A에 고정되지 않는다.
 */
class IRobotController
{
public:
    /** @brief Interface를 통한 다형 삭제를 위한 virtual destructor. */
    virtual ~IRobotController() = default;

    /**
     * @brief Backend를 사용 가능한 상태로 초기화/연결한다.
     * @return 성공 시 Result::Success(). Hardware는 transport 연결, Simulation은 내부 state 초기화를 수행한다.
     */
    virtual Result Connect() = 0;

    /**
     * @brief Backend 연결/리소스를 정리하고 더 이상 command를 받지 않는 상태로 전환한다.
     * @note noexcept 계약이므로 구현체는 정리 과정의 오류를 예외로 전파하면 안 된다.
     */
    virtual void Disconnect() noexcept = 0;

    /** @return Connect가 성공하고 backend가 command를 받을 수 있으면 true. */
    [[nodiscard]] virtual bool IsConnected() const noexcept = 0;

    /**
     * @brief Joint-space 절대 위치 이동을 요청한다.
     * @param command J1..Jn 절대 목표각 [rad]과 velocity/acceleration scale.
     * @return 관절 개수 불일치, limit 초과, 비연결 상태 등을 Result로 반환한다.
     *
     * @note command.targetPositionRadians의 길이/순서는 현재 RobotSpecification과 일치해야 한다.
     */
    virtual Result MoveJoint(const JointMoveCommand& command) = 0;

    /**
     * @brief TCP Cartesian 직선 이동을 요청한다.
     * @param command 목표 위치 [m], 방향 quaternion [x,y,z,w], 최대 선/각속도.
     * @return backend가 Cartesian motion을 지원하지 않으면 ErrorCode::Unsupported가 가능하다.
     */
    virtual Result MoveLinear(const LinearMoveCommand& command) = 0;

    /**
     * @brief 현재 software motion을 중지한다.
     * @return backend가 stop 요청을 수락했는지 나타내는 Result.
     * @warning 실제 로봇의 물리 Emergency Stop 회로 또는 safety-rated stop을 대체하지 않는다.
     */
    virtual Result Stop() = 0;

    /**
     * @brief 현재 backend의 Robot 상태 snapshot을 반환한다.
     * @return 관절 위치 [rad], 속도 [rad/s], mode/fault/TCP 정보를 포함한 값 복사본.
     */
    [[nodiscard]] virtual RobotState GetState() const = 0;

    /**
     * @brief Backend 내부 상태를 한 제어 주기 진행한다.
     * @param dtSeconds 이전 update 이후 경과 시간 [s]. 0 이하/비정상 값 처리 정책은 구현체가 정한다.
     *
     * Simulation은 target 추종/FSM을 진행하고 Hardware backend는 feedback polling/timeout 갱신 등에 사용할 수 있다.
     * Rendering FPS와 제어 주기를 분리한 뒤에도 이 함수의 dt 단위는 second로 유지한다.
     */
    virtual void Update(double dtSeconds) = 0;
};

} // namespace grasplink::robotics
