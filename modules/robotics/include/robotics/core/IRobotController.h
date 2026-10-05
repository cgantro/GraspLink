#pragma once

#include "robotics/core/ControlTypes.h"

namespace grasplink::robotics
{

/**
 * @brief 실제 장치와 Simulation backend가 공유하는 로봇 제어 계약.
 * @details
 * 관절 순서와 제한은 RobotSpecification에서 제공한다. 이 순수 domain 인터페이스는 명령과
 * feedback만 정의하며 Flecs entity나 GLM transform을 직접 다루지 않는다. 호출 성공은 요청의
 * 수락을 나타낼 수 있고 목표 도달은 GetState()로 별도 확인한다. 연결, fault, 명령 교체 정책은
 * backend마다 다르다. Joint-space 명령은 각 관절 목표각을 지정하고 Cartesian 명령은 TCP의
 * 위치와 방향을 지정한다. Backend는 이 계약 뒤에서 실제 장치 또는 Simulation을 수행한다.
 * GetState()는 특정 시점의 상태 복사본이다. software Stop은 보호 정지나 하드웨어 E-Stop을 대신하지 않는다.
 */
class IRobotController
{
public:
    virtual ~IRobotController() = default;

    /**
     * @brief backend 연결 또는 초기화를 요청한다.
     * @return 성공 또는 구체적인 실패 분류와 진단. 성공만으로 물리 장비의 안전 상태를 보장하지 않는다.
     */
    virtual Result Connect() = 0;

    /** @brief backend 자원을 정리하고 연결을 해제한다. */
    virtual void Disconnect() noexcept = 0;

    /** @brief backend가 명령을 처리할 연결 상태인지 반환한다. */
    [[nodiscard]] virtual bool IsConnected() const noexcept = 0;

    /**
     * @brief 절대 관절 목표를 요청한다.
     * @param command J1..Jn 목표각 [rad]과 backend별 속도·가속도 비율.
     * @return 요청을 받아들였는지 나타낸다. 성공은 목표 도달이나 동작 완료가 아니다.
     * @details 관절 개수·범위와 Busy 처리, 실행 중 명령 교체 여부는 backend 계약에 따른다.
     */
    virtual Result MoveJoint(const JointMoveCommand& command) = 0;

    /**
     * @brief TCP 직선 이동을 요청한다.
     * @param command TCP 위치 [m], quaternion [x,y,z,w], 선속도 상한 [m/s], 각속도 상한 [rad/s].
     * @return 수락 여부와 실패 분류. backend가 기능을 제공하지 않으면 Unsupported일 수 있다.
     * @details 실행에는 IK와 경로 실행이 필요하다. FK로 pose를 계산할 수 있어도 이 명령이 지원된다는
     * 뜻은 아니다. pose의 기준 좌표계는 이 공통 형식만으로 정해지지 않는다.
     */
    virtual Result MoveLinear(const LinearMoveCommand& command) = 0;

    /**
     * @brief backend의 software 동작 정지를 요청한다.
     * @return 요청 처리 결과. 성공은 backend 수준에서 요청을 처리했다는 뜻이다.
     * @details 물리 E-Stop이나 protective stop 회로의 동작을 보장하거나 대체하지 않는다.
     */
    virtual Result Stop() = 0;

    /**
     * @brief 현재 feedback snapshot을 값으로 반환한다.
     * @return 관절 위치 [rad], 속도 [rad/s], mode, fault 및 TCP pose [m] 복사본.
     * @details valid는 전체 snapshot, tcpPoseValid는 TCP pose만의 유효성을 나타낸다. faultCode 의미는
     * backend/장치 계약을 따른다. TCP pose가 FK의 ToolFrame 결과와 동일하다고 가정할 수 없다.
     */
    [[nodiscard]] virtual RobotState GetState() const = 0;

    /**
     * @brief backend 상태를 경과 시간만큼 진행하거나 갱신한다.
     * @param dtSeconds 경과 시간 [s]. 유효 범위와 갱신 정책은 backend 계약에 따른다.
     * @details 하드웨어 backend는 실제 제어 주기를 별도로 가질 수 있다. 이 함수가 호출됐다고 해서
     * 고정된 관절 이동량이나 동작 완료가 보장되는 것은 아니다.
     */
    virtual void Update(double dtSeconds) = 0;
};

} // namespace grasplink::robotics
