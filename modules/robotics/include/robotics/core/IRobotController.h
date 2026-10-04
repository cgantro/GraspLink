#pragma once

#include "robotics/core/ControlTypes.h"

namespace grasplink::robotics
{

/**
 * @brief 실제 Robot과 Simulation이 똑같은 방식으로 제어되도록 만드는 공통 인터페이스.
 *
 * @details
 * 여기서 Interface는 "구현은 다르지만 바깥에서 호출하는 함수 모양은 같게 만든 약속"이다.
 *
 * 예를 들어:
 * - SimRobotController: 메모리 속 관절 상태를 계산해서 움직임을 흉내냄.
 * - 향후 HanwhaHardwareController: 실제 HCR 장비에 통신 명령을 보냄.
 *
 * 두 구현 모두 상위 코드에서는 `MoveJoint()`, `Stop()`, `GetState()`처럼 같은 API로 사용한다.
 * 따라서 IK/Planner/Application은 장비가 진짜 로봇인지 시뮬레이션인지 몰라도 된다.
 *
 * 용어:
 * - Backend: 이 인터페이스 뒤에서 실제 동작을 구현하는 구체 클래스.
 * - Joint-space: J1,J2,... 관절각 목록으로 로봇 자세를 표현하는 방식.
 * - Cartesian: X/Y/Z 위치와 방향으로 로봇 끝단을 표현하는 방식.
 * - TCP: Tool Center Point. 로봇 끝단 공구의 대표 기준점.
 * - Update(dt): 시간이 dt만큼 흘렀다고 보고 내부 상태를 한 단계 진행하는 함수.
 */
class IRobotController
{
public:
    /**
     * @brief 파생 Controller를 base pointer로 삭제할 때 올바른 destructor가 호출되도록 하는 virtual destructor.
     * @note 로봇 제어 의미보다 C++ 다형성 안전성을 위한 함수다.
     */
    virtual ~IRobotController() = default;

    /**
     * @brief Controller를 명령 가능한 상태로 연결/초기화한다.
     * @return 성공 시 Result::Success(). 실패 시 이유가 담긴 Result.
     *
     * Simulation에서는 내부 상태 초기화, Hardware에서는 socket/serial/controller 연결 등을 수행할 수 있다.
     */
    virtual Result Connect() = 0;

    /**
     * @brief Controller 연결과 관련 리소스를 정리하고 더 이상 명령을 받지 않는 상태로 만든다.
     * @note noexcept이므로 정리 과정에서 예외를 밖으로 던지지 않는 계약이다.
     */
    virtual void Disconnect() noexcept = 0;

    /** @return 현재 Controller가 Connect되어 명령을 받을 수 있으면 true. */
    [[nodiscard]] virtual bool IsConnected() const noexcept = 0;

    /**
     * @brief J1..Jn의 목표각을 직접 지정해 로봇을 Joint-space로 이동시킨다.
     * @param command 각 관절의 절대 목표각 [rad]과 속도/가속도 비율.
     * @return 명령 수락 여부. joint 수 불일치, limit 초과, 비연결 상태 등은 실패 Result로 반환한다.
     *
     * @details
     * 예를 들어 HCR-12A에서 `[0.5, 0, 0, 0, 0, 0]`을 주면 J1의 목표각을 0.5 rad로 지정한다.
     * 첫 원소부터 J1,J2,... 순서이며 RobotSpecification의 joint 순서와 일치해야 한다.
     */
    virtual Result MoveJoint(const JointMoveCommand& command) = 0;

    /**
     * @brief TCP를 목표 위치/방향까지 직선 경로로 이동시키는 Cartesian 명령을 요청한다.
     * @param command 목표 TCP pose와 선/각속도 제한.
     * @return backend가 아직 Cartesian motion을 지원하지 않으면 ErrorCode::Unsupported 가능.
     *
     * @details
     * 이 함수가 실제로 동작하려면 보통 IK가 "목표 TCP pose -> 관절각"으로 변환하고,
     * trajectory 계층이 시간에 따른 경로를 만들어야 한다.
     */
    virtual Result MoveLinear(const LinearMoveCommand& command) = 0;

    /**
     * @brief 현재 소프트웨어 motion을 중단한다.
     * @return stop 요청 수락 여부.
     * @warning 실제 산업용 로봇의 Emergency Stop(E-Stop)이나 safety-rated stop 회로를 대체하지 않는다.
     */
    virtual Result Stop() = 0;

    /**
     * @brief 현재 로봇 상태를 값 복사본(snapshot)으로 가져온다.
     * @return 관절각 [rad], 관절속도 [rad/s], mode/fault/TCP 정보.
     *
     * @details 반환값은 복사본이므로 호출자가 수정해도 Controller 내부 상태가 직접 바뀌지 않는다.
     */
    [[nodiscard]] virtual RobotState GetState() const = 0;

    /**
     * @brief 시간이 dtSeconds만큼 진행됐다고 보고 backend 내부 상태를 한 단계 갱신한다.
     * @param dtSeconds 이전 Update 이후 경과 시간 [s].
     *
     * Simulation에서는 목표각을 향해 관절을 조금 이동시키고,
     * Hardware에서는 feedback polling/timeout/watchdog 갱신 등에 사용할 수 있다.
     * 향후 Fixed Control Loop를 적용하면 일정한 dt로 반복 호출하는 것이 목표다.
     */
    virtual void Update(double dtSeconds) = 0;
};

} // namespace grasplink::robotics
