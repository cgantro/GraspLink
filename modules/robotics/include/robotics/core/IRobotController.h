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

    /*
     * [추가 용어 설명]
     * - Interface: 구현 방법은 숨기고 외부에서 사용할 함수 형태만 약속하는 계약.
     * - Backend: 이 Interface 뒤에서 실제 장비 또는 Simulation을 수행하는 구현체.
     * - Joint-space motion: TCP 좌표가 아니라 각 관절의 목표각으로 움직이는 방식.
     * - Cartesian motion: TCP의 3차원 위치/방향을 기준으로 움직이는 방식.
     * - Software Stop: 프로그램이 이동 명령을 중단하는 것. 안전등급 Emergency Stop 회로와 다르다.
     * - Update(dt): dt초가 지났다고 보고 backend 내부 상태를 한 제어 step 진행하는 함수.
     */

    /**
     * @brief backend를 사용할 수 있는 상태로 초기화/연결한다.
     * @return 성공이면 Result::Success(), 실패면 오류 코드와 message를 반환한다.
     * @note Hardware 구현은 통신 연결을 열 수 있고 Simulation 구현은 내부 state를 초기화할 수 있다.
     */
    virtual Result Connect() = 0;

    /**
     * @brief backend 연결/리소스를 정리한다.
     * @note noexcept이므로 구현체는 정리 중 예외를 외부로 던지지 않아야 한다.
     */
    virtual void Disconnect() noexcept = 0;

    /** @return Connect 이후 command를 처리할 준비가 되어 있으면 true. */
    [[nodiscard]] virtual bool IsConnected() const noexcept = 0;

    /**
     * @brief J1..Jn의 절대 목표 관절각으로 이동을 요청한다.
     * @param command 목표각 [rad]과 속도/가속도 비율.
     * @note 명령 수락과 목표 도달은 같은 뜻이 아니다. 보통 실제 상태 변화는 이후 Update/feedback에서 확인한다.
     */
    virtual Result MoveJoint(const JointMoveCommand& command) = 0;

    /**
     * @brief TCP를 목표 Cartesian pose로 직선 이동시키도록 요청한다.
     * @param command 목표 위치 [m], 방향 quaternion, 최대 선/각속도.
     * @note Simulation에서 IK/trajectory가 아직 없으면 Unsupported를 반환할 수 있다.
     */
    virtual Result MoveLinear(const LinearMoveCommand& command) = 0;

    /** @warning 실제 물리 E-Stop 회로를 대체하지 않는 software motion stop이다. */
    virtual Result Stop() = 0;

    /** @return 현재 관절 위치/속도, mode, fault, TCP 등을 담은 상태 snapshot 복사본. */
    [[nodiscard]] virtual RobotState GetState() const = 0;

    /**
     * @brief backend 내부 상태를 한 제어 주기 진행한다.
     * @param dtSeconds 이전 호출 이후 경과 시간 [s].
     */
    virtual void Update(double dtSeconds) = 0;
};

} // namespace grasplink::robotics
